use embassy_executor::Spawner;
use mote_api::messages::mote_to_host;

use super::pubsub;

pub async fn init(spawner: Spawner) {
    spawner.spawn(guidance_task().expect("guidance_task already spawned"));
}

#[embassy_executor::task]
async fn guidance_task() {
    let mut context = GuidanceContext::default();
    context.execute().await;
}

#[derive(Default)]
struct GuidanceContext {}

impl GuidanceContext {
    fn handle_position_message(&mut self, _position: pubsub::Position) -> pubsub::Guidance {
        pubsub::Guidance {
            heading: units::PlaneAngle::new(0.0),
            distance: units::Length::new(0.0),
        }
    }

    async fn execute(&mut self) {
        let mut subscriber = pubsub::NOTIFY_PUBSUB.subscriber().expect("no subscriber available");
        let publisher = pubsub::NOTIFY_PUBSUB
            .publisher()
            .expect("not publisher available to guidance");
        loop {
            let incoming_data = subscriber.next_message_pure().await;
            if let pubsub::Message::Position(position) = incoming_data {
                let outgoing_message = pubsub::Message::new_from_guidance(self.handle_position_message(position));
                publisher.publish(outgoing_message).await;
            }
        }
    }
}
