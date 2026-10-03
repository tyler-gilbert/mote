use embassy_executor::Spawner;

use super::pubsub;

pub async fn init(spawner: Spawner) {
    spawner.spawn(control_task().expect("control_task already spawned"));
}

#[embassy_executor::task]
async fn control_task() {
    let mut control_context = ControlContext::default();
    control_context.execute().await;
}

#[derive(Default)]
struct ControlContext {
    control_mode_enabled: bool,
    position: pubsub::Position,
}

impl ControlContext {
    fn handle_guidance_message(&mut self, guidance_message: pubsub::Guidance) -> Option<pubsub::Control> {
        // decide to turn right or left based on the heading/distance
        if self.control_mode_enabled {
            let pubsub::Guidance { heading, distance } = guidance_message;

            Some(pubsub::Control {
                left: units::AngularVelocity::new(0.0),
                right: units::AngularVelocity::new(0.0),
            })
        } else {
            None
        }
    }

    async fn execute(&mut self) {
        let mut subscriber = pubsub::NOTIFY_PUBSUB.subscriber().expect("no subscriber available");
        let publisher = pubsub::NOTIFY_PUBSUB.publisher().expect("no publisher available");
        loop {
            let incoming_data = subscriber.next_message_pure().await;
            match incoming_data {
                pubsub::Message::EnableControlMode => self.control_mode_enabled = true,
                pubsub::Message::DisableControlMode => self.control_mode_enabled = false,
                pubsub::Message::Position(position_message) => self.position = position_message,
                pubsub::Message::Guidance(guidance_message) => {
                    if let Some(message) = self.handle_guidance_message(guidance_message) {
                        publisher.publish(pubsub::Message::new_from_control(message)).await;
                    }
                }
                _ => {}
            }
        }
    }
}
