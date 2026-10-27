import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location(
    "axis5_readback", Path(__file__).parents[1] / "apps/read_axis5_parameters.py"
)
app = importlib.util.module_from_spec(spec)
spec.loader.exec_module(app)


@pytest.mark.parametrize(
    "response_kind", ["valid", "missing", "pmac_error", "connect_error"]
)
def test_readonly_capture_and_failure_evidence(tmp_path, response_kind):
    events = []

    class Client:
        def load_system_host_keys(self):
            pass

        def set_missing_host_key_policy(self, _):
            pass

        def connect(self, **kwargs):
            events.append("connect")
            if response_kind == "connect_error":
                raise OSError("secret must never enter the record")

        def exec_command(self, command, **kwargs):
            events.append(command)
            assert "=" not in command
            assert command == app.read_command()
            response = "\n".join(q + "=1" for q in app.QUERIES)
            if response_kind == "missing":
                response = "Motor[5].Pos=1"
            elif response_kind == "pmac_error":
                response += "\nERR003"
            stdout = io.BytesIO(response.encode())
            stdout.channel = SimpleNamespace(recv_exit_status=lambda: 1)
            return None, stdout, io.BytesIO(b"")

        def close(self):
            events.append("close")

    path = tmp_path / "readback.json"
    if response_kind == "valid":
        app.capture(app.PMACConfig(), path, Client)
    else:
        with pytest.raises(RuntimeError, match="incomplete"):
            app.capture(app.PMACConfig(), path, Client)
    assert events[-1] == "close"
    record = json.loads(path.read_text())
    assert record["complete"] == (response_kind == "valid")
    assert "secret" not in path.read_text()
    assert "ssh_pass" not in record
    count = len(events)
    with pytest.raises(FileExistsError):
        app.capture(app.PMACConfig(), path, Client)
    assert len(events) == count
