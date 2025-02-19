from hydra.utils import instantiate
from typing import List, cast
from omegaconf import DictConfig

from .abstract_adversity import AbstractAdversity


def _build_adversity(adversity_cfg: DictConfig) -> AbstractAdversity:
    """Build an adversity object from the configuration.

    Args:
        adversity_cfg (DictConfig): Configuration of the adversity.

    Returns:
        AbstractAdversity: Adversity object.
    """
    config = adversity_cfg.copy()
    adversity = cast(AbstractAdversity, instantiate(config))
    return adversity


def build_adversities(adversity_cfg: DictConfig) -> List[AbstractAdversity]:
    """Build adversities from the configuration.

    Args:
        adversity_cfg (DictConfig): Configuration of the adversities.
        
    Returns:
        List[AbstractAdversity]: List of adversities.
    """
    return [_build_adversity(adversity) for adversity in adversity_cfg.values()]
