from fastapi import FastAPI, HTTPException
import multiprocessing as mp


SEGFAULT_PROCESS_RETURNCODE = -11

from examples.teaser_python_ply.teaser_python_ply import python_teaser
app = FastAPI()

@app.post("/evaluate")
async def evaluate(output: str = ""):
    # Assuming evaluate_ply_files is your existing function that takes file contents
    queue = mp.Queue()

    try:
        proc = mp.Process(target=python_teaser, args=(queue, output))
        proc.start()

        proc.join(timeout=20)

        if proc.is_alive():
            print("Subprocess timed out or encountered an error, terminating...")
            proc.terminate()

        if not queue.empty():
            result = queue.get()
        else:
            raise HTTPException(status_code=404, detail="No result")

        return {"result": result}
    except:
        raise HTTPException(status_code=404, detail="No result")