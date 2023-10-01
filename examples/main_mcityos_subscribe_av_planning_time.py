from terasim_mr.communicationtools import constants
from terasim_mr.communicationtools.mcityos_subsciber import BasicSubscriber


AV_planning_time_subscriber = BasicSubscriber(
    information_redis_key=constants.REDIS_CONSTANTS.AV_PLANNING_TIME,
    mcityos_topic_name=constants.MCITYOS_CONSTANTS.TOPIC_AV_PLANNING_TIME,
    mcityos_namespace_connection=constants.MCITYOS_CONSTANTS.NAMESPACE_TERASIM,
    mcityos_namespace_information=constants.MCITYOS_CONSTANTS.NAMESPACE_AV,
)
AV_planning_time_subscriber.run()