from terasim_mr.communicationtools import constants
from terasim_mr.communicationtools.mcityos_publisher import BasicPublisher


terasim_status_publisher = BasicPublisher(
    information_redis_key=constants.REDIS_CONSTANTS.TERASIM_STATUS,
    mcityos_topic_name=constants.MCITYOS_CONSTANTS.TOPIC_TERASIM_STATUS,
    mcityos_namespace_connection=constants.MCITYOS_CONSTANTS.NAMESPACE_TERASIM,
    frequency=10.0
)
terasim_status_publisher.run()