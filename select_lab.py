import os

Import("env")

lab_dir = os.path.join(env.subst("$PROJECT_DIR"), "labs", env["PIOENV"])
env.Replace(PROJECT_SRC_DIR=lab_dir)