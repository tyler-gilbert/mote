use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
pub use mote_api::messages::pubsub::*;

const NOTIFY_SUBS: usize = 6;
const NOTIFY_PUBS: usize = 6;
const NOTIFY_CAPACITY: usize = 4;

pub static NOTIFY_PUBSUB: PubSubChannel<CriticalSectionRawMutex, Message, NOTIFY_CAPACITY, NOTIFY_SUBS, NOTIFY_PUBS> =
    PubSubChannel::new();

pub static SCAN_CHAN: Channel<CriticalSectionRawMutex, Scan, 1> = Channel::new();

pub type NotifyPublisher =
    Publisher<'static, CriticalSectionRawMutex, Message, NOTIFY_CAPACITY, NOTIFY_SUBS, NOTIFY_PUBS>;
pub type NotifySubscriber =
    Subscriber<'static, CriticalSectionRawMutex, Message, NOTIFY_CAPACITY, NOTIFY_SUBS, NOTIFY_PUBS>;

pub type ScanSender<'a> = Sender<'a, CriticalSectionRawMutex, Scan, 1>;
