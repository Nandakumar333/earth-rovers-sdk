"""High-level mission and semantic search objective contracts."""

from typing import List, Optional
from pydantic import BaseModel, Field


class SearchTarget(BaseModel):
    """Semantic target description for open-vocabulary search."""

    target_id: str
    prompt: str = Field(description="Natural language prompt, e.g. 'red plastic canister'")
    min_confidence: float = Field(default=0.8, ge=0.0, le=1.0)
    required_observations: int = Field(default=3, ge=1)


class MissionObjective(BaseModel):
    """High-level mission parameters and objectives."""

    mission_id: str
    target: Optional[SearchTarget] = None
    search_radius_m: float = Field(default=25.0, ge=1.0)
    max_duration_s: float = Field(default=1800.0, ge=10.0)
    checkpoints: List[str] = Field(default_factory=list)
