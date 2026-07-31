import math

import torch
from mmcv.runner import HOOKS, Hook
from mmcv.runner.hooks.logger import TextLoggerHook
from tqdm.auto import tqdm


@HOOKS.register_module()
class CarlaFileLoggerHook(TextLoggerHook):
    """Keep MMCV JSON logs without flooding the terminal during training."""

    def _log_info(self, log_dict, runner):
        # CarlaProgressBarHook presents the live training metrics. The parent
        # class still writes the full log dictionary to *.log.json. Evaluation
        # summaries remain visible because the train bar is closed by then.
        # EvalHook flushes the train buffer before inference, before MMCV's
        # timer has inserted ``time``. TextLoggerHook then mislabels that flush
        # as ``val``; recognize it by its loss keys and keep it off terminal.
        contains_train_loss = (
            "loss" in log_dict
            or any(key.startswith("frame_") for key in log_dict)
        )
        if log_dict.get("mode") != "train" and not contains_train_loss:
            super()._log_info(log_dict, runner)


@HOOKS.register_module()
class CarlaProgressBarHook(Hook):
    """A compact live progress bar for the iteration-based CARLA run."""

    def __init__(self, iters_per_epoch, num_epochs, mininterval=0.25):
        self.iters_per_epoch = int(iters_per_epoch)
        self.num_epochs = int(num_epochs)
        self.mininterval = float(mininterval)
        if self.iters_per_epoch <= 0 or self.num_epochs <= 0:
            raise ValueError("iters_per_epoch and num_epochs must be positive")
        self._bar = None
        self._last_iter = 0

    @staticmethod
    def _scalar(value):
        if isinstance(value, torch.Tensor):
            return value.detach().float().item()
        try:
            return float(value)
        except (TypeError, ValueError):
            return math.nan

    @staticmethod
    def _learning_rate(runner):
        learning_rates = runner.current_lr()
        if isinstance(learning_rates, dict):
            learning_rates = next(iter(learning_rates.values()))
        return float(learning_rates[0])

    def _open(self, runner):
        if runner.rank != 0 or self._bar is not None:
            return
        self._last_iter = runner.iter
        self._bar = tqdm(
            total=runner.max_iters,
            initial=runner.iter,
            desc="Train",
            unit="iter",
            dynamic_ncols=True,
            mininterval=self.mininterval,
            smoothing=0.1,
            leave=True,
        )

    def _close(self):
        if self._bar is not None:
            self._bar.refresh()
            self._bar.close()
            self._bar = None

    def before_train_iter(self, runner):
        self._open(runner)

    def after_train_iter(self, runner):
        if runner.rank != 0:
            return
        self._open(runner)

        current_iter = runner.iter + 1
        epoch = min(
            (current_iter - 1) // self.iters_per_epoch + 1,
            self.num_epochs,
        )
        epoch_iter = (current_iter - 1) % self.iters_per_epoch + 1
        log_vars = runner.outputs.get("log_vars", {})
        loss = self._scalar(log_vars.get("loss", math.nan))
        vram_gib = (
            torch.cuda.max_memory_allocated() / (1024**3)
            if torch.cuda.is_available()
            else 0.0
        )

        self._bar.set_description_str(
            f"Epoch {epoch:02d}/{self.num_epochs:02d}"
        )
        self._bar.set_postfix(
            {
                "epoch_iter": f"{epoch_iter}/{self.iters_per_epoch}",
                "loss": f"{loss:.3f}",
                "lr": f"{self._learning_rate(runner):.2e}",
                "VRAM": f"{vram_gib:.1f}G",
            },
            refresh=False,
        )
        self._bar.update(max(0, current_iter - self._last_iter))
        self._last_iter = current_iter

        # Evaluation has its own progress bar. This hook runs above checkpoint
        # and evaluation hooks, so finish the terminal line before they start,
        # then reopen the train bar for the next epoch.
        if (
            current_iter % self.iters_per_epoch == 0
            or current_iter >= runner.max_iters
        ):
            self._close()

    def after_run(self, runner):
        self._close()
