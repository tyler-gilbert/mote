use super::pubsub;

pub async fn init(spawner: embassy_executor::Spawner) {
    spawner.spawn(guidance_task().expect("guidance_task already spawned"));
}

#[embassy_executor::task]
async fn guidance_task() {
    let mut context = GuidanceContext::default();
    context.execute().await;
}

#[derive(Default)]
struct GuidanceContext {
    context: mote_api::work::guidance::Context,
    route: pubsub::Route,
}

impl GuidanceContext {
    async fn execute(&mut self) {
        let mut subscriber = pubsub::NOTIFY_PUBSUB.subscriber().expect("no subscriber available");
        let publisher = pubsub::NOTIFY_PUBSUB
            .publisher()
            .expect("no publisher available to guidance");
        loop {
            let incoming_data = subscriber.next_message_pure().await;
            if let pubsub::Message::Position(position) = incoming_data {
                if let Some(outgoing_message) = self.context.update(pubsub::get_timestamp(), position, &self.route) {
                    publisher.publish(pubsub::Message::Guidance(outgoing_message)).await;
                }
            }
        }
    }
}
