use defmt::{error, info, warn};
use embassy_futures::select::{Either4, select4};
use embassy_net::Stack;
use embassy_net::udp::{PacketMetadata, UdpMetadata, UdpSocket};
use mote_api::HostLink;
use mote_api::messages::{host_to_mote, mote_to_host};

use crate::tasks::pubsub;
use crate::tasks::wifi::{DATA_OFFLOAD_CHANNEL, MOTOR_COMMAND_CHANNEL};

pub const UDP_SERVER_PORT: u16 = 7475;

#[embassy_executor::task]
pub(super) async fn udp_server_task(stack: Stack<'static>) -> ! {
    stack.wait_link_up().await;
    stack.wait_config_up().await;
    let mut context_zero = UdpServerContextZero::new();
    let mut context = UdpServerContext::new(stack, &mut context_zero);
    context.execute().await
}

pub(super) struct UdpServerContextZero {
    rx_meta: [PacketMetadata; 16],
    rx_buffer: [u8; 4096],
    tx_meta: [PacketMetadata; 16],
    tx_buffer: [u8; 4096],
}

impl UdpServerContextZero {
    pub(super) const fn new() -> Self {
        Self {
            rx_meta: [PacketMetadata::EMPTY; 16],
            rx_buffer: [0; 4096],
            tx_meta: [PacketMetadata::EMPTY; 16],
            tx_buffer: [0; 4096],
        }
    }
}

pub(super) struct UdpServerContext<'a> {
    socket: UdpSocket<'a>,
    publisher: pubsub::NotifyPublisher,
    subscriber: pubsub::NotifySubscriber,
    link: HostLink,
    message_buffer: [u8; 4096],
    client: Option<UdpMetadata>,
    telemetry_selection: pubsub::TelemetrySelection,
}

impl<'a> UdpServerContext<'a> {
    fn new(stack: Stack<'static>, context_zero: &'a mut UdpServerContextZero) -> Self {
        let socket = UdpSocket::new(
            stack,
            &mut context_zero.rx_meta,
            &mut context_zero.rx_buffer,
            &mut context_zero.tx_meta,
            &mut context_zero.tx_buffer,
        );

        let publisher = pubsub::NOTIFY_PUBSUB
            .publisher()
            .expect("not pubsub publisher for UDP server task");
        let subscriber = pubsub::NOTIFY_PUBSUB
            .subscriber()
            .expect("not pubsub subscriber for UDP server task");

        Self {
            socket,
            publisher,
            subscriber,
            link: HostLink::new(),
            message_buffer: [0; 4096],
            client: None,
            telemetry_selection: pubsub::TelemetrySelection::None,
        }
    }

    async fn execute(&mut self) -> ! {
        if let Err(e) = self.socket.bind(UDP_SERVER_PORT) {
            warn!("bind error: {:?}", e);
        }

        loop {
            match select4(
                self.socket.recv_from(&mut self.message_buffer),
                DATA_OFFLOAD_CHANNEL.receive(),
                self.subscriber.next_message_pure(),
                pubsub::SCAN_PUBLISH_CHAN.receive(),
            )
            .await
            {
                Either4::First(Ok((bytes_read, ep))) => self.handle_received_datagram(bytes_read, ep).await,
                Either4::First(Err(err)) => {
                    error!("UDP recv error: {}", err);
                }
                Either4::Second(message) => self.handle_data_offload(message).await,
                Either4::Third(pubsub_message) => self.handle_local_pubsub_message(pubsub_message).await,
                Either4::Fourth(scan_message) => {
                    self.send_to_client(mote_to_host::Message::PubSub(
                        mote_api::messages::pubsub::Message::LidarScan(scan_message),
                    ))
                    .await
                }
            }
        }
    }

    async fn handle_received_datagram(&mut self, bytes_read: usize, endpoint: UdpMetadata) {
        let new_client = match self.client {
            None => {
                info!("Client connected: {}", endpoint);
                true
            }
            Some(ref current) if *current != endpoint => {
                info!("Client changed: {} -> {}", current, endpoint);
                true
            }
            _ => false,
        };
        if new_client {
            self.client = Some(endpoint);
        }

        self.link.handle_receive(&self.message_buffer[..bytes_read]);
        loop {
            match self.link.poll_receive() {
                Ok(Some(message)) => {
                    self.handle_command(message).await;
                }
                Ok(None) => break,
                Err(mote_api::Error::VersionMismatch {
                    local,
                    remote,
                    local_role,
                    remote_role,
                    behind,
                }) => {
                    warn!(
                        "Dropped message: mote-api version mismatch ({} v{}.{}.{}, {} v{}.{}.{}) — update {}",
                        local_role.as_str(),
                        local.major,
                        local.minor,
                        local.patch,
                        remote_role.as_str(),
                        remote.major,
                        remote.minor,
                        remote.patch,
                        behind.as_str(),
                    );
                }
                Err(_) => warn!("Dropped undecodable message"),
            }
        }

        while let Some(payload) = self.link.poll_transmit() {
            if let Err(err) = self.socket.send_to(&payload, endpoint).await {
                error!("UDP send error: {}", err);
            }
        }
    }

    async fn handle_local_pubsub_message(&mut self, message: pubsub::Message) {
        match message {
            pubsub::Message::Position(position) => {
                if self.telemetry_selection.is_send_navigation() {
                    self.send_to_client(mote_to_host::Message::PubSub(pubsub::Message::Position(position)))
                        .await;
                }
            }
            pubsub::Message::Guidance(guidance) => {
                if self.telemetry_selection.is_send_guidance() {
                    self.send_to_client(mote_to_host::Message::PubSub(pubsub::Message::Guidance(guidance)))
                        .await;
                }
            }
            pubsub::Message::Control(control_message) => {
                if self.telemetry_selection.is_send_control() {
                    let message = host_to_mote::SetDriveBaseVelocity {
                        left_velocity_rad_per_s: control_message.left.into(),
                        right_velocity_rad_per_s: control_message.right.into(),
                    };
                    MOTOR_COMMAND_CHANNEL.send(message).await;
                    self.send_to_client(mote_to_host::Message::PubSub(pubsub::Message::Control(control_message)))
                        .await;
                }
            }
            pubsub::Message::LidarScan(value) => {
                if self.telemetry_selection.is_send_lidar() {
                    self.send_to_client(mote_to_host::Message::PubSub(pubsub::Message::LidarScan(value)))
                        .await;
                }
            }
            pubsub::Message::Imu(value) => {
                if self.telemetry_selection.is_send_imu() {
                    self.send_to_client(mote_to_host::Message::PubSub(pubsub::Message::Imu(value)))
                        .await;
                }
            }
            _ => {
                defmt::error!("Unhandled local pubsub: {}", message);
            }
        }
    }

    async fn handle_data_offload(&mut self, message: mote_to_host::Message) {
        self.send_to_client(message).await;
    }

    async fn send_to_client(&mut self, message: mote_to_host::Message) {
        if let Some(endpoint) = self.client {
            self.link.send(message).unwrap();

            while let Some(payload) = self.link.poll_transmit() {
                if let Err(err) = self.socket.send_to(&payload, endpoint).await {
                    if matches!(err, embassy_net::udp::SendError::NoRoute) {
                        info!("Client disconnected: {}", endpoint);
                        self.client = None;
                    } else {
                        error!("UDP send error: {}", err);
                    }
                    break;
                }
            }
        }
    }

    async fn handle_pubsub_message_from_host(&mut self, message: pubsub::Message) {
        match &message {
            pubsub::Message::TelemetrySelection(telemetry_selection) => {
                defmt::info!("Setting Telemetry Selection: {}", telemetry_selection);
                self.telemetry_selection = telemetry_selection.clone();
                self.publisher
                    .publish(pubsub::Message::TelemetrySelection(telemetry_selection.clone()))
                    .await;
            }
            pubsub::Message::EnableControlMode | pubsub::Message::DisableControlMode => {
                self.publisher.publish(message).await;
            }
            _ => {
                defmt::error!("Unhandled remote pubusb: {}", message);
            }
        }
    }

    async fn handle_command(&mut self, rx_message: host_to_mote::Message) {
        match rx_message {
            host_to_mote::Message::Ping => {
                info!("Parsed ping request, responding.");
                let _ = self.link.send(mote_to_host::Message::Pong);
            }
            host_to_mote::Message::Pong => {
                info!("Received ping response from host.");
            }
            host_to_mote::Message::SetDriveBaseVelocity(cmd) => {
                let _ = MOTOR_COMMAND_CHANNEL.try_send(cmd);
            }
            host_to_mote::Message::PubSub(message) => {
                self.handle_pubsub_message_from_host(message).await;
            }
            _ => {
                error!("Received unhandled message type");
            }
        }
    }
}
