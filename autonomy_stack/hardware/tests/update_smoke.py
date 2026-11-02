"""Verify in-place deployment migration with real, network-isolated Docker containers."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import uuid

import yaml


def run(*args,**kwargs):
    return subprocess.check_output(args,text=True,stderr=subprocess.STDOUT,**kwargs)


parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--image',default='ghcr.io/bedathenic/ranger-autonomy:jazzy-r3')
args=parser.parse_args()
hardware=Path(__file__).resolve().parents[1]
token=uuid.uuid4().hex[:8]
with tempfile.TemporaryDirectory(prefix='ranger-update-test-') as tmp:
    root=Path(tmp);deployment=root/'deployment';deployment.mkdir()
    payload=root/'payload';payload.mkdir();originals=root/'originals';originals.mkdir()
    helpers=root/'bin';helpers.mkdir()
    environment='AUTONOMY_IMAGE='+args.image+'\nNAV_PROFILE=holonomic\nUSER_LOCAL_VALUE=preserve-me\n'
    (deployment/'.env').write_text(environment)
    (deployment/'maps').mkdir();(deployment/'maps/keep.txt').write_text('local map')
    (deployment/'logs').mkdir();(deployment/'logs/keep.txt').write_text('local logs')
    shutil.copyfile(hardware/'apply_update.sh',deployment/'apply_update.sh')
    files=['compose.yaml','compose.gui.yaml','compose.drive.yaml','drive_guard.py','robot.sh','viewer.sh',
           'drive.sh','inspect_drive.py','ackermann_diagnostics.py','ackermann_check.sh','README.md']
    for name in files:shutil.copyfile(hardware/name,payload/name)
    configurations=[('compose.yaml','planner'),('compose.gui.yaml','rviz-web'),('compose.drive.yaml','drive-guard')]
    configs=[];containers=[];before={}
    try:
        for index,(name,role) in enumerate(configurations):
            project='ranger-update-test-'+token+'-'+str(index)
            old=dict(name=project,services={role:dict(image=args.image,entrypoint=['sleep','infinity'],
                                                     init=True,network_mode='none')})
            (originals/name).write_text(json.dumps(old))
            (deployment/name).write_text(json.dumps(old))
            new=yaml.safe_load((payload/name).read_text());new['name']=project
            for service,definition in new['services'].items():
                definition['container_name']='ranger_hero_test_'+token+'_'+service
            (payload/name).write_text(json.dumps(new))
            compose=['docker','compose','--project-directory',str(deployment),'-f',str(originals/name)]
            configs.append(compose)
            run(*compose,'up','-d')
            cid=run(*compose,'ps','--quiet',role).strip();containers.append(cid)
            info=json.loads(run('docker','inspect',cid))[0]
            before[cid]=info['State']['StartedAt']
            run('docker','exec',cid,'sh','-c','printf local-controller-edit > /tmp/local-controller.cpp')
        (deployment/'drive.sh').write_text('#!/usr/bin/env bash\nset -eu\n[[ "$1" == disarm ]]\nprintf called >> "$MOCK_DISARM_LOG"\n')
        # Stage current checked-out files rather than making the test depend on GitHub availability.
        (helpers/'curl').write_text('#!/usr/bin/env python3\nimport os,sys,shutil\nfrom pathlib import Path\n'
            'args=sys.argv[1:]; url=next(a for a in args if a.startswith("https://")); '
            'target=args[args.index("-o")+1]; shutil.copyfile(Path(os.environ["UPDATE_TEST_PAYLOAD"])/url.rsplit("/",1)[1],target)\n')
        (helpers/'curl').chmod(0o755)
        env=dict(os.environ,PATH=str(helpers)+':'+os.environ['PATH'],UPDATE_TEST_PAYLOAD=str(payload),
                 MOCK_DISARM_LOG=str(root/'disarmed'))
        output=run('bash',str(deployment/'apply_update.sh'),'0'*40,env=env)
        assert (deployment/'.env').read_text()==environment
        assert (deployment/'maps/keep.txt').read_text()=='local map'
        assert (deployment/'logs/keep.txt').read_text()=='local logs'
        assert (root/'disarmed').read_text()=='called'
        for cid,(_,role) in zip(containers,configurations):
            info=json.loads(run('docker','inspect',cid))[0]
            assert info['Name']=='/ranger_hero_test_'+token+'_'+role,info['Name']
            assert info['State']['Running']
            assert run('docker','exec',cid,'cat','/tmp/local-controller.cpp')=='local-controller-edit'
            if role!='drive-guard':assert info['State']['StartedAt']==before[cid],'Application unexpectedly restarted'
            else:assert info['State']['StartedAt']!=before[cid],'Gateway did not reload its code'
        assert list((deployment/'update_backups').glob('*/.env'))
        print('PASS: real container IDs and application writable layers preserved; only gateway restarted; settings/maps/logs kept')
    finally:
        for compose in configs:
            subprocess.run(compose+['down','--remove-orphans'],stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,timeout=30)
