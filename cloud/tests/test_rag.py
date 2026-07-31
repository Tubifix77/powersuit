"""RAG ingest/retrieval over the live operator manual (docs/link-protocol.md §7:
"RAG retrieve (top-k over docs/manuals/)"). Ingests the real
cloud/docs/manuals/suit_manual.md — the same corpus the server loads by
default — and checks retrieval quality and determinism.
"""

from __future__ import annotations

from pathlib import Path

from cloud_ai_core.config import Settings
from cloud_ai_core.rag.ingest import chunk_text, ingest_dir

MANUALS_DIR = Path(Settings().rag_docs_dir)


def test_manuals_dir_is_the_default_settings_location() -> None:
    # Sanity check that we're testing against the exact corpus the server loads.
    assert MANUALS_DIR.is_dir()
    assert (MANUALS_DIR / "suit_manual.md").is_file()


def test_ingest_produces_a_nonempty_store() -> None:
    store = ingest_dir(MANUALS_DIR)
    assert len(store) > 0


def test_battery_query_retrieves_battery_section() -> None:
    store = ingest_dir(MANUALS_DIR)
    hits = store.search("battery care thresholds overvoltage undervoltage", k=3)
    assert hits, "expected at least one hit"
    assert hits[0].score > 0.0
    combined = "\n".join(h.text.lower() for h in hits[:1])
    assert "battery" in combined
    assert "overvoltage" in combined or "cell" in combined


def test_estop_query_retrieves_estop_section() -> None:
    store = ingest_dir(MANUALS_DIR)
    hits = store.search("how do I clear an e-stop with the monotonic counter", k=3)
    assert hits
    combined = "\n".join(h.text.lower() for h in hits[:2])
    assert "estop" in combined or "e-stop" in combined


def test_determinism_across_two_independent_ingests() -> None:
    store_a = ingest_dir(MANUALS_DIR)
    store_b = ingest_dir(MANUALS_DIR)
    query = "battery state of charge low warning"
    hits_a = store_a.search(query, k=5)
    hits_b = store_b.search(query, k=5)
    assert [(h.doc_id, h.text, round(h.score, 6)) for h in hits_a] == [
        (h.doc_id, h.text, round(h.score, 6)) for h in hits_b
    ]


def test_determinism_within_a_single_process_repeat_query() -> None:
    store = ingest_dir(MANUALS_DIR)
    query = "airbrake flap deploy"
    first = store.search(query, k=3)
    second = store.search(query, k=3)
    assert [(h.doc_id, h.score) for h in first] == [(h.doc_id, h.score) for h in second]


def test_chunker_rejects_overlap_ge_size() -> None:
    import pytest

    with pytest.raises(ValueError):
        chunk_text("x" * 100, size=100, overlap=100)


def test_missing_docs_dir_yields_empty_store(tmp_path) -> None:
    store = ingest_dir(tmp_path / "does-not-exist")
    assert len(store) == 0
    assert store.search("anything") == []
