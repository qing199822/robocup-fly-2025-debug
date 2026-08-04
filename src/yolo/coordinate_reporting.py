def publish_actor_info_with_heartbeat(
    actor_publisher,
    actor_message,
    heartbeat_publisher,
    heartbeat_message,
):
    actor_publisher.publish(actor_message)
    heartbeat_publisher.publish(heartbeat_message)
