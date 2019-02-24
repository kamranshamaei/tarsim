from inc import *
import threading


COM_THREAD_INTERVAL_S = COM_THREAD_INTERVAL_MS / 1000


class ComminucationModule:
    def __init__(self, joint_index_dict=dict(), joint_value_dict=dict()):
        self.joint_index_dict = joint_index_dict
        self.joint_value_dict = joint_value_dict
        self.com_thread = threading.Thread(target=self.com_thread_function)
        self.com_thread_time_regulator = TimeRegulator(COM_THREAD_INTERVAL_S)
        self.com_thread_time_interval = 0
        self.com_thread.start()

    def com_thread_function(self):
        while True:
            self.receive_joint_values()
            self.send_poses()

            # Regulate the time
            self.com_thread_time_interval = self.com_thread_time_regulator.regulate_time()

    def receive_joint_values(self):
        for key in self.joint_index_dict:
            self.joint_index_dict[key].mate_to_parent.value.put(10*time.clock())

    def send_poses(self):
        pass


