from fastapi import FastAPI, HTTPException
import multiprocessing as mp
import time


SEGFAULT_PROCESS_RETURNCODE = -11

from examples.teaser_python_ply.teaser_python_ply import python_teaser, python_teaser_original

app = FastAPI()

@app.post("/evaluate")
async def evaluate():
    queue = mp.Queue()
    timeout_seconds = 20  # Main timeout for the subprocess
    check_interval = 0.1  # Interval to check if the process has completed

    try:
        proc = mp.Process(target=python_teaser, args=(queue, ))
        print("Starting subprocess")
        proc.start()

        # Wait for the process to finish with a timeout
        elapsed_time = 0
        while elapsed_time < timeout_seconds:
            if not proc.is_alive():
                break
            time.sleep(check_interval)
            elapsed_time += check_interval

        if proc.is_alive():
            print("Subprocess timed out, terminating...")
            proc.terminate()
            proc.join()

        # Wait a bit before checking the queue
        results = []
        additional_timeout = 5  # Additional timeout for queue check (in seconds)
        waited = 0
        while waited < additional_timeout:
            if not queue.empty():
                while not queue.empty():
                    results.append(queue.get())
                break
            time.sleep(check_interval)
            waited += check_interval

        if not results:
            results = ["Subprocess did not return any result"]

        return {"results": results}
    except Exception as e:
        print(f"Exception in FastAPI endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/evaluate-original")
async def evaluate_original():
    queue = mp.Queue()
    timeout_seconds = 20  # Main timeout for the subprocess
    check_interval = 0.1  # Interval to check if the process has completed

    try:
        proc = mp.Process(target=python_teaser_original, args=(queue, ))
        print("Starting subprocess")
        proc.start()

        # Wait for the process to finish with a timeout
        elapsed_time = 0
        while elapsed_time < timeout_seconds:
            if not proc.is_alive():
                break
            time.sleep(check_interval)
            elapsed_time += check_interval

        if proc.is_alive():
            print("Subprocess timed out, terminating...")
            proc.terminate()
            proc.join()

        # Wait a bit before checking the queue
        results = []
        additional_timeout = 5  # Additional timeout for queue check (in seconds)
        waited = 0
        while waited < additional_timeout:
            if not queue.empty():
                while not queue.empty():
                    results.append(queue.get())
                break
            time.sleep(check_interval)
            waited += check_interval

        if not results:
            results = ["Subprocess did not return any result"]

        return {"results": results}
    except Exception as e:
        print(f"Exception in FastAPI endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))