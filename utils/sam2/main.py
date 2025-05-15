import os
import sys
from utils.config import Config
from sam2_processor import Sam2

def setup_logging():
    import logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    return logging.getLogger(__name__)

def main():
    """Main entry point of the application."""
    logger = setup_logging()
    
    logger.info("Loading configuration...")
    cfg = Config()

    logger.info("Initializing SAM2 processor...")
    seg_pcd = Sam2(cfg)
    
    if cfg.do_annotation:
        logger.info("Starting annotation process...")
        seg_pcd.init_model(ann_frame_idx=0)

    if cfg.do_generalize:
        logger.info("Starting frame processing...")
        seg_pcd.generalize()
        
    logger.info("SAM2 processing completed successfully")

    


if __name__ == '__main__':
    main()