import json

from glob import glob
from collections import defaultdict

def get_split_files(split='test',env='gibson'):
    dirs = glob('/home/faith/GitRepos/habitat/'+env.lower()+'_test_splits/*/*.json')
    env_file_list = defaultdict(list)
    for d in dirs:
        with open(d) as f:
            files = json.load(f)
        key = f.name.split('splits/')[-1].split('.')[0]
        for e in files['episodes']:
            env_file_list[key].append(e['scene_id'].split('/')[-1])

    temp=[]
    for key in env_file_list.keys():
        env_file_list[key]=set(env_file_list[key])
        temp.extend(env_file_list[key])
    env_file_list['all']=set(temp)

    if split=='train':
        breakpoint()

    return env_file_list