from typing import Iterable

from mmcv.runner import HOOKS, Hook


@HOOKS.register_module()
class FreezeModelModulesHook(Hook):
    """Freeze named model submodules while leaving task heads trainable.

    IterBasedRunner calls ``model.train()`` before every iteration. Therefore
    frozen modules are returned to eval mode in ``before_train_iter`` as well
    as having their parameters disabled once in ``before_run``.
    """

    def __init__(self, module_names: Iterable[str], set_eval=True):
        self.module_names = tuple(module_names)
        self.set_eval = bool(set_eval)
        if not self.module_names:
            raise ValueError("module_names must not be empty")
        self._modules = []

    @staticmethod
    def _model(runner):
        return getattr(runner.model, "module", runner.model)

    @staticmethod
    def _resolve(root, dotted_name):
        module = root
        for component in dotted_name.split("."):
            if not hasattr(module, component):
                raise AttributeError(
                    f"Cannot freeze {dotted_name!r}: missing {component!r}"
                )
            module = getattr(module, component)
        return module

    def before_run(self, runner):
        model = self._model(runner)
        frozen_parameters = 0
        self._modules = []
        for name in self.module_names:
            module = self._resolve(model, name)
            self._modules.append(module)
            for parameter in module.parameters():
                parameter.requires_grad_(False)
                frozen_parameters += parameter.numel()
            if self.set_eval:
                module.eval()

        trainable_parameters = sum(
            parameter.numel()
            for parameter in model.parameters()
            if parameter.requires_grad
        )
        runner.logger.info(
            "Frozen modules %s: %s parameters frozen, %s parameters trainable",
            list(self.module_names),
            f"{frozen_parameters:,}",
            f"{trainable_parameters:,}",
        )

    def before_train_iter(self, runner):
        if self.set_eval:
            for module in self._modules:
                module.eval()
