# WindowTEC VLM 窗户四角点检测

这是一个可独立运行的 VLM 窗户四角点检测项目。脚本将图片和提示词发送给 OpenAI-compatible 视觉模型，要求模型仅返回窗户外框四角点，并生成 JSONL、SVG 标注和 HTML 人工复查页。

当前版本是经过四轮测试后保留的 `boxes-only` 版本，不调用 YOLO、SAM 或其他分割模型。

## 环境要求

- Python 3.10 或更高版本；
- 一个支持图片输入的 OpenAI-compatible VLM 服务；
- 服务需要接受 User Message 中的 `image_url` Data URL；
- 推荐支持严格 `response_format.json_schema`。

项目只使用 Python 标准库，不需要安装第三方依赖。

## 文件说明

```text
windowtec_vlm_window_corners/
├── run.py                  # 主程序
├── system_prompt.md        # 最终 System Prompt
├── task_prompt.md          # 最终窗户标注规则
├── windowtec_core/         # 随包内置的请求、JSON 和文件工具
├── tests/                  # 单元测试
├── images/                 # 默认输入目录
├── PACKAGE_INFO.json       # 版本和输出协议
└── requirements.txt        # 无第三方依赖
```

## 快速开始

1. 将待处理的 `.png`、`.jpg` 或 `.jpeg` 图片放入 `images/`。
2. 设置 API key。不要把真实密钥写进项目文件：

```bash
export WINDOWTEC_API_KEY='<your-api-key>'
```

3. 在项目根目录执行：

```bash
python3 run.py \
  --base-url http://127.0.0.1:18000/v1 \
  --model auto
  --api-key diff
```

`--model auto` 会先请求 `/v1/models` 并使用返回的第一个模型，也可以明确指定模型：

```bash
python3 run.py \
  --base-url http://127.0.0.1:8000/v1 \
  --model Qwen3.6-35B-A3B-UN-NVFP4
```

如果服务不校验 API key，可不设置环境变量；脚本会发送占位值 `EMPTY`。

## 输入方式

支持图片目录、单张图片或 ZIP 压缩包：

```bash
python3 run.py --input images
python3 run.py --input /path/to/image.png
python3 run.py --input /path/to/images.zip
```

ZIP 会安全解压到本次结果目录，只读取 `.png`、`.jpg` 和 `.jpeg`。

## 模型请求协议

每张图片单独请求一次 `/v1/chat/completions`：

- System Message：`system_prompt.md`；
- User Message：图片名称、原始尺寸、`task_prompt.md`、坐标协议和 JSON 示例；
- 图片：Base64 Data URL，格式为 `data:image/png;base64,...`；
- 坐标：`0..1000` 整数；
- 四角顺序：左上、右上、右下、左下；
- 模型只允许输出 `windows[].points`。

期望的模型原始输出：

```json
{
  "windows": [
    {
      "points": [[183, 456], [283, 456], [283, 631], [183, 631]]
    }
  ]
}
```

如果模型服务拒绝自定义 `response_format`，脚本会自动用相同提示词和图片、不带 Schema 重试一次。也可以显式使用 `--no-response-format`。

## 输出文件

默认输出到 `results/vlm_window_corners/`：

- `predictions.jsonl`：模型原始响应、解析后的 `points_1000`、内部 `0..1` 坐标、像素坐标和校验结果；
- `annotated/*.svg`：包含原图和四角点 polygon 的自包含 SVG；
- `review.html`：整组图片的可视化人工复查页；
- `summary.json`：请求成功数、格式通过数、窗户总数和延迟汇总。

处理完成后直接在浏览器打开：

```text
results/vlm_window_corners/review.html
```

## 常用参数

```bash
# 指定输出目录
python3 run.py --out-dir results/my_run

# 只处理前 3 张图片
python3 run.py --limit 3

# 从已成功且格式有效的图片后继续
python3 run.py --resume

# 不请求模型，只重新生成已有结果的 HTML/SVG
python3 run.py --render-only --out-dir results/my_run

# 使用合成框检查解析和可视化链路
python3 run.py --dry-run --limit 1
```

查看全部参数：

```bash
python3 run.py --help
```

## 运行测试

```bash
python3 -m py_compile run.py windowtec_core/*.py tests/*.py
python3 -m unittest discover -s tests -v
```

## 更换检测对象

保持 `system_prompt.md`、坐标协议和 JSON Schema 不变，修改 `task_prompt.md` 中的目标定义和筛选规则即可。无论检测什么对象，每个目标仍需输出四个有序角点。

## 已知限制

- VLM 返回的是视觉语义驱动的近似角点，不是像素级几何测量；
- 提示词不能保证模型完全不推测角点；
- 没有人工标注真值时，窗户数量和框形状不能作为准确率；
- 对像素级精度有要求时，需要增加人工真值，并考虑后续几何校正或关键点模型。
