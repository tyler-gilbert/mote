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
    full_scan: heapless::Deque<pubsub::ScanPoint, 5000>,
}

impl NavigationContext {
    fn handle_incoming_scan(&mut self, scan: pubsub::Scan) -> Option<pubsub::Position> {
        for point in scan {
            if self.full_scan.is_full() {
                let _ = self.full_scan.pop_front();
            }
            let _ = self.full_scan.push_back(point);
        }

        if self.full_scan.is_full() {
            for point in self.full_scan.iter() {}
        }

        None
    }

    fn handle_incoming_imu(&mut self, _imu: pubsub::Imu) -> Option<pubsub::Position> {
        None
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
                    pubsub::Message::Imu(imu) => self.handle_incoming_imu(imu),
                    _ => None,
                },
                Either::Second(scan) => self.handle_incoming_scan(scan),
            };
            if let Some(position) = position {
                publisher.publish(pubsub::Message::new_from_position(position)).await;
            }
        }
    }
}
