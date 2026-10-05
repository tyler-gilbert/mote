use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};

use super::pubsub;

pub async fn init(spawner: Spawner) {
    spawner.spawn(navigation_task().expect("navigation_task already spawned"));
}

#[embassy_executor::task]
async fn navigation_task() {
    let mut context = NavigationContext::default();
    context.execute().await;
}

#[derive(Default)]
struct NavigationContext {
    context: mote_api::work::navigation::Context,
    telemetry_selection: pubsub::TelemetrySelection,
}

impl NavigationContext {
    async fn handle_incoming_scan(&mut self, scan: pubsub::Scan) -> Option<pubsub::Position> {
        if self.telemetry_selection.is_send_lidar() {
            pubsub::SCAN_PUBLISH_CHAN.send(scan.clone()).await;
        }
        self.context.update(pubsub::get_timestamp(), Some(scan), None)
    }

    async fn execute(&mut self) {
        let mut subscriber = pubsub::NOTIFY_PUBSUB
            .subscriber()
            .expect("no subscriber available for navigation");
        let publisher = pubsub::NOTIFY_PUBSUB
            .publisher()
            .expect("no publisher available to navigation");
        let scan_receiver = pubsub::SCAN_CHAN.receiver();
        loop {
            let incoming = select(subscriber.next_message_pure(), scan_receiver.receive()).await;
            let position = match incoming {
                Either::First(incoming_data) => match incoming_data {
                    pubsub::Message::TelemetrySelection(telemetry_selection) => {
                        self.telemetry_selection = telemetry_selection;
                        None
                    }
                    pubsub::Message::Imu(imu) => self.context.update(pubsub::get_timestamp(), None, Some(imu)),
                    _ => None,
                },
                Either::Second(scan) => self.handle_incoming_scan(scan).await,
            };
            if let Some(position) = position {
                publisher.publish(pubsub::Message::new_from_position(position)).await;
            }
        }
    }
}
