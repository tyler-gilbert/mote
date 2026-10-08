use embassy_time::{Duration, with_timeout};

use super::router;

pub async fn init(spawner: embassy_executor::Spawner) {
    spawner.spawn(gnc_task().expect("gnc_task already spawned"));
}

#[embassy_executor::task]
async fn gnc_task() {
    let mut context = Context::default();
    context.execute().await;
}

#[derive(Default)]
struct Context {
    guidance_context: mote_api::work::guidance::Context,
    control_context: mote_api::work::control::Context,
    navigation_context: mote_api::work::navigation::Context,
    route: Option<router::Route>,
    control_mode_enabled: bool,
    position: Option<router::Position>,
    reference_frame: Option<router::ReferenceFrame>,
    telemetry_selection: router::TelemetrySelection,
}

impl Context {
    async fn handle_lidar_scan(&mut self, scan: router::Scan) -> Option<router::Position> {
        if self.telemetry_selection.is_send_lidar() {
            let _ = router::TO_WIFI_CHAN.try_send(router::Message::LidarScan(scan.clone()));
        }
        if let Some(reference_frame) = self.reference_frame.as_ref() {
            self.navigation_context.update(
                router::get_timestamp(),
                self.position.as_ref(),
                Some(scan),
                None,
                reference_frame,
            )
        } else {
            None
        }
    }

    async fn handle_imu(&mut self, imu: router::Imu) -> Option<router::Position> {
        if self.telemetry_selection.is_send_imu() {
            let _ = router::TO_WIFI_CHAN.try_send(router::Message::Imu(imu.clone()));
        }
        if let Some(reference_frame) = self.reference_frame.as_ref() {
            self.navigation_context.update(
                router::get_timestamp(),
                self.position.as_ref(),
                None,
                Some(imu),
                reference_frame,
            )
        } else {
            None
        }
    }

    async fn send_position(&self, position: router::Position) {
        if self.telemetry_selection.is_send_navigation() {
            let _ = router::TO_WIFI_CHAN.try_send(router::Message::Position(position));
        }
    }

    async fn send_motor_command(&self, motor_drive: router::MotorDrive) {
        if self.control_mode_enabled {
            router::MOTOR_COMMAND_CHANNEL.send(motor_drive.into()).await;
        }
    }

    async fn send_guidance(&self, guidance: Option<&router::Guidance>) {
        if self.telemetry_selection.is_send_guidance()
            && let Some(guidance) = guidance
        {
            let _ = router::TO_WIFI_CHAN.try_send(router::Message::Guidance(guidance.clone()));
        }
    }

    async fn send_control(&self, control: router::Control) {
        if self.telemetry_selection.is_send_control() {
            let _ = router::TO_WIFI_CHAN.try_send(router::Message::Control(control.clone()));
        }
    }

    async fn handle_update(&mut self) {
        let timestamp = router::get_timestamp();
        if let (Some(route), Some(position)) = (self.route.as_ref(), self.position.as_ref()) {
            let guidance = self.guidance_context.update(timestamp, position, route);
            self.send_guidance(guidance.as_ref()).await;
            if let Some(control) = self.control_context.update(timestamp, guidance) {
                self.send_control(control.clone()).await;
                self.send_motor_command(control.motor_drive).await;
            }
        }
    }

    async fn handle_message(&mut self, message: router::Message) {
        let position = match message {
            router::Message::None => None,
            router::Message::EnableControlMode => {
                self.control_mode_enabled = true;
                None
            }
            router::Message::DisableControlMode => {
                self.control_mode_enabled = false;
                None
            }
            router::Message::TelemetrySelection(telemetry_selection) => {
                self.telemetry_selection = telemetry_selection;
                None
            }
            router::Message::ReferenceFrame(reference_frame) => {
                self.reference_frame = Some(reference_frame);
                None
            }
            router::Message::MotorDrive(motor_drive) => {
                self.send_motor_command(motor_drive).await;
                None
            }
            router::Message::Imu(imu) => self.handle_imu(imu).await,
            router::Message::LidarScan(scan) => self.handle_lidar_scan(scan).await,
            router::Message::Position(position) => {
                self.position = Some(position);
                None
            }
            _ => None,
        };
        if let Some(position) = position {
            self.send_position(position.clone()).await;
            self.position = Some(position);
        }
        self.handle_update().await;
    }

    async fn execute(&mut self) {
        loop {
            let timeout_result = with_timeout(Duration::from_millis(50), router::TO_GNC_CHAN.receive()).await;
            match timeout_result {
                Ok(message) => self.handle_message(message).await,
                Err(_) => self.handle_update().await,
            }
        }
    }
}
