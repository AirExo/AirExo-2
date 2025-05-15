# ➡️ Dataset Transformation

After data collection, we need to transform the data into suitable format for downstream policy learning. Please refer to [the data format](../../README.md#-data-format) for more details.

Use the following commands to transform the dataset into the required format.

```bash
python -m airexo.adaptor.dataset_transform +path=/path/to/the/task
```

Other parameters are specified in [the configuration file](../../airexo/configs/adaptor/dataset_transform.yaml).
