"""MY-LIVO shared Python tooling (dataset access, playback, ROS environment).

Import from repository scripts with::

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
    from livo import rosenv, dataset

Nothing in this package depends on the SLAM sources; it only reads MCAP recordings and
files under results/ and config/.
"""
__version__ = "2.0.0"
