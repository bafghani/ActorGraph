warning('Create your own test suite in the test directory')

test_actorgraph_exe = executable('test_ActorGraph.executable', 
    sources: ['test_ActorGraph.cpp'], 
    dependencies : [actorgraph_dep, gtest_dep])
test('my actorgraph test', test_actorgraph_exe)

test_map_exe = executable('test_Map.executable', 
    sources: ['test_Map.cpp'], 
    dependencies : [map_dep, gtest_dep])
test('my map test', test_map_exe)