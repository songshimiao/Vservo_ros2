import sys
import types
import pytest
from vservo_ros2.vendor.camera_stream import OrbbecColorStream


def test_separate_process_selection_opens_only_its_camera(monkeypatch):
    opened = []
    class StopAfterSelection(Exception):
        pass
    class Device:
        def get_device_info(self):
            return self
        def get_name(self):
            return 'Orbbec Gemini 335'
        def get_serial_number(self):
            return 'ETH'
    class Devices:
        def get_count(self):
            return 2
        def get_device_name_by_index(self, i):
            return ['Gemini 305', 'Gemini 335'][i]
        def get_device_serial_number_by_index(self, i):
            return ['EIH', 'ETH'][i]
        def get_device_by_index(self, i):
            opened.append(i)
            assert i == 1, 'Do not acquire another process camera'
            return Device()
    class Context:
        def query_devices(self):
            return Devices()
    def pipeline(device):
        raise StopAfterSelection()
    monkeypatch.setitem(sys.modules, 'pyorbbecsdk', types.SimpleNamespace(Context=Context, Pipeline=pipeline))
    with pytest.raises(StopAfterSelection):
        OrbbecColorStream(serial='ETH', model='gemini335')
    assert opened == [1]
