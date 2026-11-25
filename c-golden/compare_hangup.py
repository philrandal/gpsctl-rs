import os, subprocess, sys, time, signal
HERE=sys.argv[3]
def go(tool):
    fake=subprocess.Popen([sys.executable, HERE+"/fake_any.py", "/tmp/hup.log"], stdout=subprocess.PIPE, text=True)
    path=fake.stdout.readline().strip()
    p=subprocess.Popen([tool,"-p",path,"--shared_port","-vv","-e"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    time.sleep(1.5); fake.kill(); fake.wait()
    try: out,err=p.communicate(timeout=30)
    except subprocess.TimeoutExpired: p.kill(); out,err=p.communicate(); out+="<<TIMEOUT>>"
    return p.returncode, out.replace(path,"@PORT"), err.replace(path,"@PORT")
a=go(sys.argv[1]); b=go(sys.argv[2])
print("C   :",a[0]); print(a[1]); print("Rust:",b[0]); print(b[1]); print("SAME" if a==b else "DIFFERENT", a[2]!=b[2] and ("stderr "+repr(a[2])+" / "+repr(b[2])) or "")
