use bsw_util::spsc::Queue;

#[test]
fn producer_consumer_preserve_order_across_wraps() {
    const COUNT: usize = 20_000;
    let mut queue = Queue::<usize, 8>::new();
    let (producer, consumer) = queue.split();

    std::thread::scope(|scope| {
        scope.spawn(move || {
            for value in 0..COUNT {
                let mut pending = value;
                loop {
                    match producer.try_push(pending) {
                        Ok(()) => break,
                        Err(value) => {
                            pending = value;
                            std::hint::spin_loop();
                        }
                    }
                }
            }
        });

        scope.spawn(move || {
            for expected in 0..COUNT {
                loop {
                    if let Some(actual) = consumer.try_pop() {
                        assert_eq!(actual, expected);
                        break;
                    }
                    std::hint::spin_loop();
                }
            }
        });
    });
}

#[test]
fn capacity_one_handoff_covers_full_and_empty_transitions() {
    let mut queue = Queue::<u32, 2>::new();
    let (producer, consumer) = queue.split();
    let first = std::sync::Arc::new(std::sync::Barrier::new(2));
    let second = std::sync::Arc::new(std::sync::Barrier::new(2));
    std::thread::scope(|scope| {
        let first_producer = first.clone();
        let second_producer = second.clone();
        scope.spawn(move || {
            assert_eq!(producer.try_push(7), Ok(()));
            assert_eq!(producer.try_push(8), Err(8));
            first_producer.wait();
            second_producer.wait();
            assert_eq!(producer.try_push(9), Ok(()));
        });
        scope.spawn(move || {
            first.wait();
            assert_eq!(consumer.try_pop(), Some(7));
            assert_eq!(consumer.try_pop(), None);
            second.wait();
            let actual = loop {
                if let Some(value) = consumer.try_pop() {
                    break value;
                }
                std::hint::spin_loop();
            };
            assert_eq!(actual, 9);
        });
    });
}
