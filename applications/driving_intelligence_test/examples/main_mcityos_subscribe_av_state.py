from terasim_mr.communicationtools import constants
from terasim_mr.communicationtools.mcityos_subsciber import BasicSubscriber


AV_state_subscriber = BasicSubscriber(
    information_redis_key=constants.REDIS_CONSTANTS.AV_STATE,
    mcityos_topic_name=constants.MCITYOS_CONSTANTS.TOPIC_AV_STATE,
    mcityos_namespace_connection=constants.MCITYOS_CONSTANTS.NAMESPACE_TERASIM,
    mcityos_namespace_information=constants.MCITYOS_CONSTANTS.NAMESPACE_AV,
)
AV_state_subscriber.run()