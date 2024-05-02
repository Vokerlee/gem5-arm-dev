# Copyright (c) 2024 Roman Glaz <vokerlee@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.defines import buildEnv
from m5.objects import *

CPU0_VD = ["1.0V"]
CPU0_CD = [
    "2000MHz",
    "1866MHz",
    "1720MHz",
    "1588MHz",
    "1433MHz",
    "1250MHz",
    "1111MHz",
    "997MHz",
    "840MHz",
    "700MHz",
    "500MHz",
]

L3_VD = ["1.0V"]
L3_CD = [
    "1222MHz",
    "1111MHz",
    "888MHz",
    "777MHz",
    "555MHz",
    "444MHz",
    "333MHz",
    "222MHz",
]

LPDDR5_VD = ["1.0V"]
LPDDR5_CD = [
    "800MHz",
    "700MHz",
    "600MHz",
    "500MHz",
    "400MHz",
    "300MHz",
    "200MHz",
]
