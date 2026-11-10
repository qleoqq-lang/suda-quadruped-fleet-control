from __future__ import annotations

from http import HTTPStatus
import os
from pathlib import Path
from typing import Iterable


class AsrError(RuntimeError):
    pass


def load_dotenv(project_dir: str | Path) -> None:
    path = Path(project_dir) / ".env"
    if not path.exists():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, value = stripped.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def resolve_api_key(project_dir: str | Path | None = None, explicit: str | None = None) -> str:
    if explicit:
        key = explicit.strip()
    else:
        if project_dir is not None:
            load_dotenv(project_dir)
        key = os.getenv("DASHSCOPE_API_KEY", "").strip()
    if not key:
        raise AsrError("未找到 DASHSCOPE_API_KEY，请在 .env 或环境变量中配置")
    return key


def recognize_file(
    path: str | Path,
    *,
    model: str = "paraformer-realtime-v2",
    api_key: str | None = None,
    language_hints: Iterable[str] = ("zh", "en"),
) -> str:
    """Transcribe one local audio file with DashScope realtime ASR."""
    import dashscope
    from dashscope.audio.asr import Recognition, RecognitionCallback

    if api_key is None:
        api_key = os.getenv("DASHSCOPE_API_KEY", "").strip()
    if not api_key:
        raise AsrError("未找到 DASHSCOPE_API_KEY，请在 .env 或环境变量中配置")
    dashscope.api_key = api_key
    callback = RecognitionCallback()
    recognition = Recognition(
        model=model,
        callback=callback,
        format="wav",
        sample_rate=16000,
        language_hints=list(language_hints),
    )
    result = recognition.call(str(Path(path)))
    if result.status_code != HTTPStatus.OK:
        raise AsrError(f"语音识别失败: code={result.code} message={result.message}")
    sentences = result.get_sentence() or []
    texts = [sentence.get("text", "") for sentence in sentences if isinstance(sentence, dict)]
    return "".join(texts).strip()
