import csv

from epucklib.trace import CsvTrace, trace_path


def test_trace_path_names_the_file_after_the_controller(tmp_path):
    path = trace_path("epuck_roomba", tmp_path)
    assert path.name == "epuck_roomba.csv"
    assert path.parent == tmp_path / "analysis" / "traces"
    assert path.parent.is_dir()  # created eagerly, so opening the file works


def test_rows_are_written_with_a_header(tmp_path):
    path = tmp_path / "trace.csv"
    with CsvTrace(path, ["t_s", "mode"]) as writer:
        writer.write(t_s=0.0, mode="SWEEP")
        writer.write(t_s=0.5, mode="GAP_FILL")

    rows = list(csv.DictReader(path.open()))
    assert [row["mode"] for row in rows] == ["SWEEP", "GAP_FILL"]
    assert rows[0]["t_s"] == "0.0"


def test_every_row_is_flushed_immediately(tmp_path):
    # Webots is killed rather than shut down at the end of a batch run, so a
    # buffered final row would simply be lost.
    path = tmp_path / "trace.csv"
    writer = CsvTrace(path, ["value"])
    writer.write(value=1)
    assert "1" in path.read_text()
    writer.close()


def test_closing_twice_is_harmless(tmp_path):
    writer = CsvTrace(tmp_path / "trace.csv", ["value"])
    writer.close()
    writer.close()
    assert writer.closed
