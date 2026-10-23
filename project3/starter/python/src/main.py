"""Project 3 foreground runtime. All graded nodes use the TCP/JSON hub."""
import sys
import time

from node_client import NodeClient
from robot_state import RobotStatePublisher
from rosbridge import Gateway
from services import ParameterServer
from world_state import RobotWorldStatePublisher


def main():
    gateway = Gateway()
    gateway.start()
    parameter_client = NodeClient()
    parameter_server = ParameterServer(parameter_client)
    robot_client = NodeClient()
    robot_state = RobotStatePublisher(robot_client)
    world_client = NodeClient()
    world_state = RobotWorldStatePublisher(world_client)
    demo_nodes = []
    if "--demo" in sys.argv[1:]:
        from joint_state_publisher import JointStatePublisher
        from finite_state_machine import FiniteStateMachine
        joint_client = NodeClient()
        fsm_client = NodeClient()
        demo_nodes = [(JointStatePublisher(joint_client), joint_client),
                      (FiniteStateMachine(fsm_client), fsm_client)]
    print("Project 3 runtime ready on 127.0.0.1:9095", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        robot_state.running = False
        world_state.running = False
        for node, client in demo_nodes:
            node.running = False
            if hasattr(node, "active"):
                node.active = False
            client.close()
        parameter_client.close()
        robot_client.close()
        world_client.close()
        gateway.close()


if __name__ == "__main__":
    main()
