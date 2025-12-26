from IPPerfMonitor import IPPerfMonitor
import ResultCollection
resultList = list()

def planning(plannerFactory, testList):
    for key,producer in list(plannerFactory.items()):
        print(key, producer)
        for benchmark in testList:
            print ("Planning: " + key + " - " + benchmark.name)
            planner = producer[0](benchmark.collisionChecker)
            IPPerfMonitor.clearData()
            try:
                resultList.append(ResultCollection.ResultCollection(key,
                                            planner, 
                                            benchmark, 
                                            planner.planPath(benchmark.startList,benchmark.goalList,producer[1]),
                                            IPPerfMonitor.dataFrame()
                                            ),
                            )
            except Exception as e:
                print ("PLANNING ERROR ! ", e)
                pass
    return resultList