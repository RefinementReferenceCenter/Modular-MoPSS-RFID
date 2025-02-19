Import("env")
# Do not run a script when external applications, such as IDEs,
# dump integration data. Otherwise, input() will block the process
# waiting for the user input
if env.IsIntegrationDump():
    # stop the current script execution
    Return()

# Ask user name

def before_upload(source, target, env):
    print("Ready For Next Module:", (env["PIOENV"]) )
    user = input()

env.AddPreAction("upload", before_upload)
