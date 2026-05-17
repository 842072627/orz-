import json
d = json.load(open('orz_V4_samezone_test.json'))
print('numTaskFinished:', d['numTaskFinished'])
print('keys:', sorted(d.keys()))
print('task_count from tasks list:', len(d.get('tasks',[])))
ts = d.get('tasks', [])
if ts:
    first_task = ts[0]
    print('first task type:', type(first_task))
    if isinstance(first_task, dict):
        print('first task keys:', list(first_task.keys()))
        print('first task:', first_task)
    elif isinstance(first_task, list):
        print('first task list:', first_task)
    elif isinstance(first_task, int):
        print('first task int:', first_task)
        print('last task int:', ts[-1])
