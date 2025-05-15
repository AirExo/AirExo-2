import os
import sys
import hydra
from omegaconf import DictConfig, OmegaConf
from hydra.utils import to_absolute_path
from pytorch_lightning.callbacks import ModelCheckpoint
import pytorch_lightning as pl
from torch.utils.data import DataLoader

os.environ["NCCL_DEBUG"] = "INFO"
os.environ["CUDA_LAUNCH_BLOCKING"] = "1"
os.environ["TORCH_DISTRIBUTED_DEBUG"] = "INFO"
os.environ['NCCL_P2P_DISABLE'] = '1'

controlnet_path = '/home/jingjing/workspace/sp1/controlnet/ControlNet/'
sys.path.append(controlnet_path)

from share import *
from cldm.logger import ImageLogger
from cldm.model import create_model, load_state_dict
from dataset import RoboticArmDataset

def setup_model(cfg):
    yaml_path = cfg.model.yaml_path
    if not yaml_path:
        yaml_path = os.path.join(controlnet_path, 'models/cldm_v15.yaml')
    else:
        yaml_path = to_absolute_path(yaml_path)
    resume_path = to_absolute_path(cfg.model.resume_path)
    
    model = create_model(yaml_path).cpu()
    model.load_state_dict(load_state_dict(resume_path, location='cpu'))
    model.learning_rate = cfg.training.learning_rate
    model.sd_locked = cfg.model.sd_locked
    model.only_mid_control = cfg.model.only_mid_control
    
    return model

def setup_callbacks(cfg):
    output_dir = to_absolute_path(cfg.paths.output_dir)
    os.makedirs(output_dir, exist_ok=True)
    
    checkpoint_callback = ModelCheckpoint(
        dirpath=output_dir,
        save_top_k=-1,
        every_n_train_steps=cfg.training.save_steps,
    )
    
    logger = ImageLogger(batch_frequency=cfg.training.logger_freq)
    
    return [checkpoint_callback, logger]

def setup_dataloader(cfg):
    camera_dir = to_absolute_path(cfg.paths.camera_dir)
    render_dir = to_absolute_path(cfg.paths.render_dir)
    
    dataset = RoboticArmDataset(
        camera_dir=camera_dir,
        render_dir=render_dir,
        prompt=cfg.training.prompt,
        img_size=cfg.training.img_size
    )
    
    dataloader = DataLoader(
        dataset,
        num_workers=cfg.training.num_workers,
        batch_size=cfg.training.batch_size,
        shuffle=True
    )
    
    return dataloader

@hydra.main(config_path="configs", config_name="train")
def main(cfg: DictConfig):
    model = setup_model(cfg)
    callbacks = setup_callbacks(cfg)
    dataloader = setup_dataloader(cfg)
    
    output_dir = to_absolute_path(cfg.paths.output_dir)
    trainer = pl.Trainer(
        default_root_dir=output_dir,
        devices=-1,
        accelerator="gpu",
        strategy="ddp",
        precision=cfg.training.precision,
        callbacks=callbacks,
        accumulate_grad_batches=cfg.training.accumulate_grad_batches,
        sync_batchnorm=True
    )
    
    trainer.fit(model, dataloader)

if __name__ == "__main__":
    main()