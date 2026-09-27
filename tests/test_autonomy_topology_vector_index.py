"""Tests for in-memory VectorIndex."""

import numpy as np
import pytest

from autonomy.topology.vector_index import VectorIndex


def test_vector_index_add_and_search():
    index = VectorIndex(dimension=4)
    v1 = [1.0, 0.0, 0.0, 0.0]
    v2 = [0.0, 1.0, 0.0, 0.0]
    index.add("node_1", v1)
    index.add("node_2", v2)

    assert len(index) == 2

    # Query identical to v1
    results = index.search([1.0, 0.0, 0.0, 0.0], top_k=2)
    assert len(results) == 2
    assert results[0][0] == "node_1"
    assert pytest.approx(results[0][1], abs=1e-4) == 1.0
    assert results[1][0] == "node_2"
    assert pytest.approx(results[1][1], abs=1e-4) == 0.0


def test_vector_index_top_k_ordering():
    index = VectorIndex(dimension=3)
    index.add("north", [0.0, 1.0, 0.0])
    index.add("north_east", [0.5, 0.5, 0.0])
    index.add("south", [0.0, -1.0, 0.0])

    # Query north
    results = index.search([0.0, 1.0, 0.0], top_k=3)
    assert len(results) == 3
    assert results[0][0] == "north"
    assert results[1][0] == "north_east"
    assert results[2][0] == "south"
    assert results[0][1] > results[1][1] > results[2][1]


def test_vector_index_threshold_filter():
    index = VectorIndex(dimension=2)
    index.add("close", [1.0, 0.1])
    index.add("orthogonal", [0.0, 1.0])

    # Only return similarity >= 0.8
    results = index.search([1.0, 0.0], top_k=5, min_similarity=0.8)
    assert len(results) == 1
    assert results[0][0] == "close"


def test_vector_index_dimension_mismatch():
    index = VectorIndex(dimension=4)
    with pytest.raises(ValueError):
        index.add("bad", [1.0, 2.0])


def test_vector_index_remove_and_clear():
    index = VectorIndex(dimension=2)
    index.add("a", [1.0, 0.0])
    index.add("b", [0.0, 1.0])

    assert index.remove("a") is True
    assert len(index) == 1
    assert index.get("a") is None
    assert index.get("b") is not None

    index.clear()
    assert len(index) == 0
    assert index.search([1.0, 0.0]) == []
