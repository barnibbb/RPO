import time
import numpy as np
import cupy as cp



if __name__ == '__main__':
    
    N = 100_000_000
    a_cpu = np.random.rand(N).astype(np.float32)
    b_cpu = np.random.rand(N).astype(np.float32)

    # CPU computation
    start_cpu = time.time()
    c_cpu = a_cpu * b_cpu + np.sqrt(a_cpu)
    cpu_time = time.time() - start_cpu


    # GPU computation
    a_gpu = cp.asarray(a_cpu)
    b_gpu = cp.asarray(b_cpu)

    cp.cuda.Stream.null.synchronize()

    start_gpu = time.time()
    c_gpu = a_gpu * b_gpu + np.sqrt(a_gpu)
    cp.cuda.Stream.null.synchronize()
    gpu_time = time.time() - start_gpu

    print(f"CPU time: {cpu_time:.4f}s, GPU time: {gpu_time:.4f}s, Speedup: {cpu_time/gpu_time:.2f}x")

