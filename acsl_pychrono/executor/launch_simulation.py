import acsl_pychrono.config.config as Cfg
import acsl_pychrono.executor as Executor

def launchSimulation():
  if Cfg.MissionConfig.wrapper_flag:
    Executor.runParallelBatch(max_parallel=Cfg.MissionConfig.wrapper_max_parallel)
  else:
    Executor.runSingleSimulation()