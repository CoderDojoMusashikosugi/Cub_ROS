from cub_simulation.topic_probe import main


def test_smoke_entrypoint_is_callable():
    assert callable(main)
