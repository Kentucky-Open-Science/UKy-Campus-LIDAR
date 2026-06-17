"""Declarative scenarios + train/test seed splits for campus_gym.

A `Scenario` is a small, shareable spec (agent type, goal kind, region, NPC traffic,
signals, domain randomization, horizon, reward weights). `make()` builds the matching
env, so reproducible test cases are data, not code. A few named scenarios ship in
`SCENARIOS`; pair them with `train_test_seeds()` for held-out evaluation.

    from campus_gym.scenarios import make_scenario, SCENARIOS, train_test_seeds
    env = make_scenario("campus_traffic")
    train, test = train_test_seeds()        # disjoint seed sets
"""
from dataclasses import dataclass, field, asdict
from typing import Optional


@dataclass
class Scenario:
    name: str = "campus"
    agent_type: str = "car"
    goal: str = "random"                 # "random" point | "named" (language-conditioned)
    region: tuple = (0.0, 0.0, 200.0)
    npc_traffic: int = 0
    signals: bool = False
    domain_random: bool = False
    max_episode_steps: int = 1000
    reward_weights: Optional[dict] = field(default=None)

    def make(self, **overrides):
        from .env import CampusEnv
        from .tasks import CampusNavEnv
        kw = dict(agent_type=self.agent_type, npc_traffic=self.npc_traffic,
                  signals=self.signals, domain_random=self.domain_random,
                  max_episode_steps=self.max_episode_steps, reward_weights=self.reward_weights)
        kw.update(overrides)
        if self.goal == "named":
            return CampusNavEnv(**kw)
        kw["region"] = self.region
        return CampusEnv(**kw)

    def to_dict(self):
        return asdict(self)


SCENARIOS = {
    # name                 -> Scenario(...)
    "campus_easy":     Scenario("campus_easy", "car", "random", region=(0, 0, 150)),
    "campus_traffic":  Scenario("campus_traffic", "car", "random", region=(0, 0, 150),
                                npc_traffic=14, signals=True),
    "delivery_named":  Scenario("delivery_named", "truck", "named", npc_traffic=10, signals=True,
                                domain_random=True, max_episode_steps=1500),
    "robot_courier":   Scenario("robot_courier", "robot", "named", npc_traffic=8, signals=True,
                                max_episode_steps=1500),
    "drone_survey":    Scenario("drone_survey", "drone", "named", max_episode_steps=1500),
}


def make_scenario(name_or_scenario, **overrides):
    if isinstance(name_or_scenario, Scenario):
        return name_or_scenario.make(**overrides)
    if name_or_scenario not in SCENARIOS:
        raise KeyError(f"unknown scenario '{name_or_scenario}'; have: {', '.join(SCENARIOS)}")
    return SCENARIOS[name_or_scenario].make(**overrides)


def train_test_seeds(n_train=200, n_test=50, test_base=1_000_000):
    """Disjoint seed sets so held-out evaluation never overlaps training."""
    return list(range(n_train)), list(range(test_base, test_base + n_test))
