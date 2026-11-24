# 下列最小设备方法仅用于替换 SDK 查询，均不访问 USB。
# 用假的 Orbbec 设备列表验证分进程采集时只取得本路相机，避免占用另一节点设备。
import sys
import types
import pytest
from vservo_ros2.vendor.camera_stream import OrbbecColorStream


# 枚举时 305 在前、335 在后；ETH 节点只能打开索引 1，并在管线初始化处停止测试。
def test_separate_process_selection_opens_only_its_camera(monkeypatch):
    opened = []
    # 测试哨兵异常：设备选择验证完成后中断，避免模拟不相关 SDK 采集接口。
    class StopAfterSelection(Exception):
        pass
    # 最小设备信息替身，返回固定型号和 ETH 序列号。
    class Device:
        # 设备对象本身兼作信息对象。
        def get_device_info(self):
            return self
        # 返回固定 335 型号名。
        def get_name(self):
            return 'Orbbec Gemini 335'
        # 返回本路 ETH 的测试序列号。
        def get_serial_number(self):
            return 'ETH'
    # 设备列表替身；get_device_by_index 会记录真正取得的设备索引。
    class Devices:
        # 模拟两台设备同时枚举。
        def get_count(self):
            return 2
        # 按索引仅读名称，不取得设备使用权。
        def get_device_name_by_index(self, i):
            return ['Gemini 305', 'Gemini 335'][i]
        # 按索引仅读序列号，帮助先筛选目标。
        def get_device_serial_number_by_index(self, i):
            return ['EIH', 'ETH'][i]
        # 记录取得设备的动作，断言不能占用索引 0 的另一台相机。
        def get_device_by_index(self, i):
            opened.append(i)
            assert i == 1, 'Do not acquire another process camera'
            return Device()
    # 把相机上下文查询替换为测试列表。
    class Context:
        # 返回假设备列表。
        def query_devices(self):
            return Devices()
    # 选中目标设备后抛哨兵异常，不建立真实采集管线。
    def pipeline(device):
        raise StopAfterSelection()
    monkeypatch.setitem(sys.modules, 'pyorbbecsdk', types.SimpleNamespace(Context=Context, Pipeline=pipeline))
    with pytest.raises(StopAfterSelection):
        OrbbecColorStream(serial='ETH', model='gemini335')
    assert opened == [1]
