#!/usr/bin/env bash
# Keep the perception process on PyTorch's bundled CUDA libraries. A system
# toolkit earlier in LD_LIBRARY_PATH can otherwise mix cuBLAS and cuBLASLt.
set -euo pipefail
sam3_python="$1"
shift
sam3_cuda_libs="$("$sam3_python" -c 'import pathlib, sys; print(":".join(dict.fromkeys(str(p) for root in sys.path if root for p in pathlib.Path(root).glob("nvidia/*/lib") if p.is_dir())))')"
if [[ -n "$sam3_cuda_libs" ]]; then
    export LD_LIBRARY_PATH="$sam3_cuda_libs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
exec "$sam3_python" "$@"
