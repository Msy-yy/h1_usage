#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
#

#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of MYBOTSHOP GmbH.

import os
import inflect

# -------------------------------------------- Local Audio Gen
import asyncio
import edge_tts
async def _async_stream_tts_playsound(text: str, filename):
    output_dir = "./src/mybotshop/go2_webserver/go2_webserver/audio"
    os.makedirs(output_dir, exist_ok=True)

    communicate = edge_tts.Communicate(
        text, voice="en-US-GuyNeural", rate="-10%")
    # Add 'await' to ensure the coroutine runs
    await communicate.save(os.path.join(output_dir, filename))

# -------------------------------------------- Local Audio Gen
# import torch
# from TTS.api import TTS

# device = "cuda" if torch.cuda.is_available() else "cpu"
# tts = TTS(model_name='tts_models/en/jenny/jenny')

# def mozilla_predefined_audio_generator(gtext, filename):
#     output_dir = "./src/mybotshop/go2_webserver/go2_webserver/audio"
#     os.makedirs(output_dir, exist_ok=True)

#     tts.tts_to_file(text=gtext, file_path=os.path.join(output_dir, filename))

# mozilla_predefined_audio_generator("Obstruction detected", "obstruction.mp3")

asyncio.run(_async_stream_tts_playsound("I have received your request. Give me a few seconds to process it!", "llm_response.mp3"))

