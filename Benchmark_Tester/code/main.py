import test_handler

def main():
    # Create a BenchmarkRunner instance with chosen parameters.
    # - model: which LLM to benchmark
    # - max_len: number of dataset samples to use per run
    # - temp: LLM temperature (controls randomness)
    # - path: where results will be saved
    testHandler = test_handler.BenchmarkRunner(
        model="llama3.2", 
        max_len=10000,#maximum number of samples to use from each dataset
        temp=0.0, 
        path="results/llama32"
    )
    # Run a specific benchmark defined in the config file
    # testHandler.run_test("web_questions_simp")
    # testHandler.run_test("web_questions_eng")
    # Run all benchmarks defined in the config file
    testHandler.run_all_benchmarks()
    testHandler.run_all_benchmarks()
    testHandler.run_all_benchmarks()

if __name__ == "__main__":
    # Entry point for running the script directly
    main()



