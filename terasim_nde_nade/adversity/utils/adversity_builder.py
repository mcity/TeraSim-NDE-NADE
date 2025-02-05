from typing import List, cast

from hydra.utils import instantiate
from omegaconf import DictConfig

from terasim_nde_nade.adversity.abstract_adversity import AbstractAdversity


def _build_adversity(adversity_cfg: DictConfig) -> AbstractAdversity:
    config = adversity_cfg.copy()
    adversity = cast(AbstractAdversity, instantiate(config))
    return adversity


def build_adversities(adversity_cfg: DictConfig) -> List[AbstractAdversity]:
    return [_build_adversity(adversity) for adversity in adversity_cfg.values()]
