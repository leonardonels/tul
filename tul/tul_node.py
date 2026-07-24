import os
import signal
import yaml
from rclpy.node import Node
import rclpy
from std_msgs.msg import Int8
from modules.imodule import IModule, AsState
from modules import REGISTRY

class TulNode(Node):
    def __init__(self):
        super().__init__('tul_node')
        
        # ====== params ======
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)['tul']
        self._debug: bool = config['debug']
        self._start_state: AsState = config['start_state']
        self._auto_shutdown: bool = config.get('auto-shutdown', False)
        self._state_topic: str = config['as_state_topic']
        self._modules_config: list[dict] = config.get('modules', [])
        self.get_logger().info("============ Configuration ===========")
        self._modules: list[IModule] = self._build_modules()
        self.get_logger().info(f"Loaded {len(self._modules)} modules")
        self.get_logger().info(f"Modules: {[module.__class__.__name__ for module in self._modules]}")
        self.get_logger().info("======================================")
        
        # ====== config ======
        self._as_subscriber = self.create_subscription(Int8, self._state_topic, self.state_callback, 1)
        self._current_state = self._start_state

    def _build_modules(self) -> list[IModule]:
        modules: list[IModule] = []
        for mod_cfg in self._modules_config:
            cls = REGISTRY.get(mod_cfg['type'])
            if cls is None:
                self.get_logger().warn(f"Unknown module type: {mod_cfg['type']}")
                continue
            module = cls(debug=self._debug, start_state=self._start_state, config=mod_cfg, node=self)
            module._jump_start()
            modules.append(module)
        return modules

    def state_callback(self, msg: Int8) -> None:
        try:
            new_state = AsState(msg.data)
        except ValueError:
            if self._debug:
                self.get_logger().warn(f"Unknown state value: {msg.data}")
            return
        
        if self._current_state == new_state:
            if self._debug:
                self.get_logger().info(f"Received state {new_state} but it's the same as current state. Ignoring.")
            return

        if new_state == AsState.EMERGENCY:  # there is no need to send a SINGINT for each EMERGENCY call, so this section can stay after the new state check
            if self._debug:
                self.get_logger().warn("Received EMERGENCY state. Stopping all modules.")
            
            if self._auto_shutdown:
                os.kill(os.getpid(), signal.SIGINT)    
        
        self._current_state = new_state
        if self._debug:
            self.get_logger().info(f"Received state: {self._current_state}")

        for module in self._modules:
            try:
                self.get_logger().info(f"Notifying module {module.__class__.__name__}: State changed to {self._current_state}")
                module.on_state_change(self._current_state)
            except Exception as e:
                self.get_logger().error(f"Error occurred while notifying module {module.__class__.__name__}: {e}")
                pass

    def destroy_node(self) -> None:
        for module in self._modules:
            try:
                module._module_stop()
            except Exception as e:
                self.get_logger().error(f'Error stopping module {module.__class__.__name__}: {e}')

        super().destroy_node()

def main():
    rclpy.init()
    node = TulNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
