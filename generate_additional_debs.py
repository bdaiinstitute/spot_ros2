#!/usr/bin/env python3

"""
A code generator to generate the additional_deb entries for bosdyn_msgs (and possibly other things).
"""

import hashlib
import requests

def sha256_of_url(url):
    sha256 = hashlib.sha256()
    with requests.get(url, stream=True) as r:
        r.raise_for_status()
        for chunk in r.iter_content(chunk_size=8192):
            if chunk:  # filter out keep-alive chunks
                sha256.update(chunk)
    return sha256.hexdigest()


VERSION = "5.0.0"
INCREMENT = "-0"
BASE_URL = f"https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/{VERSION}/"
DEBIAN_NAME = "jammy"
PACKAGES = [
    "bosdyn-api-msgs",
    "bosdyn-auto-return-api-msgs",
    "bosdyn-autowalk-api-msgs",
    "bosdyn-cmake-module",
    "bosdyn-graph-nav-api-msgs",
    "bosdyn-keepalive-api-msgs",
    "bosdyn-log-status-api-msgs",
    "bosdyn-metrics-logging-api-msgs",
    "bosdyn-mission-api-msgs",
    "bosdyn-msgs",
    "bosdyn-spot-api-msgs",
    "bosdyn-spot-cam-api-msgs",
]
ARCHS = ["amd64", "arm64"]

HUB_NAME = "humble"

ENTRY_TEMPLATE = """\
ros.additional_deb(
    hub_name = '{hub_name}',
    name = '{name}',
    urls_by_arch = {urls_by_arch},
    sha256_by_arch = {sha256_by_arch},
    dependencies = [],
    cc_includes = [],
)
"""

print("Generating. Downloading debians to compute their sha hashes, be a little patient.\n")

output = []
# for each entry
for pkg in PACKAGES:
    # for each architecture
    urls_by_arch = {}
    sha256_by_arch = {}
    for arch in ARCHS:
        # construct the url
        url = BASE_URL+"ros-humble-"+pkg+"_"+VERSION+INCREMENT+DEBIAN_NAME+"_"+arch+".deb"
        print(f"downloading and computing hash for {url}...")
        urls_by_arch[arch] = [url]
        # get the sha
        try:
            sha = sha256_of_url(url)
        except requests.exceptions.HTTPError as e:
            print(f"WARNING: HTTPError on url {url}:\n{e}\n")
            continue
        sha256_by_arch[arch] = sha
    # construct entry
    entry = ENTRY_TEMPLATE.format(
        hub_name = HUB_NAME, 
        name = pkg, 
        urls_by_arch = str(urls_by_arch),
        sha256_by_arch = str(sha256_by_arch)
    )
    output.append(entry)

# proto2ros has a different version
# I'm sure there's a way to not repeat myself, but it's not worth refactoring to do so (yet)

PACKAGES = ["proto2ros",]
VERSION = "1.0.0"

for pkg in PACKAGES:
    # for each architecture
    urls_by_arch = {}
    sha256_by_arch = {}
    for arch in ARCHS:
        # construct the url
        url = BASE_URL+"ros-humble-"+pkg+"_"+VERSION+INCREMENT+DEBIAN_NAME+"_"+arch+".deb"
        print(f"downloading and computing hash for {url}...")
        urls_by_arch[arch] = [url]
        # get the sha
        try:
            sha = sha256_of_url(url)
        except requests.exceptions.HTTPError as e:
            print(f"WARNING: HTTPError on url {url}:\n{e}\n")
            continue
        sha256_by_arch[arch] = sha
    # construct entry
    entry = ENTRY_TEMPLATE.format(
        hub_name = HUB_NAME, 
        name = pkg, 
        urls_by_arch = str(urls_by_arch),
        sha256_by_arch = str(sha256_by_arch)
    )
    output.append(entry)

print("\n***** COPY THE FOLLOWING TO YOUR MODULE.bazel file *****\n")
print("\n".join(output))