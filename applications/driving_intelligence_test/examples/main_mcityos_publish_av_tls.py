from terasim_mr.communicationtools import constants
from terasim_mr.communicationtools.mcityos_publisher import BasicPublisher


AV_tls_publisher = BasicPublisher(
    information_redis_key=constants.REDIS_CONSTANTS.AV_TLS,
    mcityos_topic_name=constants.MCITYOS_CONSTANTS.TOPIC_AV_TLS,
    mcityos_namespace_connection=constants.MCITYOS_CONSTANTS.NAMESPACE_TERASIM,
    frequency=10.0
)
AV_tls_publisher.run()