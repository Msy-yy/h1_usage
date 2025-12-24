#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.

# -------------------------------------------- TTS + LLM 
import os
import glob
import asyncio
import threading
import edge_tts
from playsound import playsound
from google import genai
from google.genai import types
from pydub import AudioSegment
import speech_recognition as sr

class ROS2LLM():
    
    def __init__(self, ros2_node):

        self.ros2_node = ros2_node
        try:
            self.client = genai.Client(api_key=os.getenv('GEMINI_API_KEY'))
            self.ros2_node.get_logger().info(self.ros2_node.colorize(
                "GenAI API Key Found", "green")
            )
        except:
            self.ros2_node.get_logger().info("GenAI API Key Not Found")

        # -------------------------------------------- Online TTS
        self.tts_loop = asyncio.new_event_loop()
        self.tts_thread = threading.Thread(
            target=self._run_tts_loop, daemon=True)
        self.tts_thread.start()
        
    def driver(self):
        try:
            latest_recording_path = self.get_latest_recording()
            if latest_recording_path:
                self.ros2_node.play_audio("llm_response.mp3")
                self.ros2_node.get_logger().info(
                    f"Processing latest recording: {latest_recording_path}")
                transcribed_text = self.transcribe_audio(
                    latest_recording_path)
                if transcribed_text:
                    self.ros2_node.get_logger().info(self.ros2_node.colorize(
                        f"Request: {transcribed_text}", "yellow"))
                    response = self.client.models.generate_content(
                        model="gemini-2.0-flash-exp",
                        contents=transcribed_text,  
                        config=types.GenerateContentConfig(
                            tools=[self.move_robot, ]
                        )
                    )
                    self.ros2_node.get_logger().info(self.ros2_node.colorize(
                        f"Response: {response.text}", "green"))
                    self.stream_tts_playsound(response.text)
                else:
                    self.ros2_node.get_logger().warning("Could not transcribe audio.")
            else:
                self.ros2_node.get_logger().warning("No recording found to process.")
        except Exception as e:
            self.ros2_node.get_logger().error(
                f"Gemini Model Error: {e}")
    
    # -------------------------------------------- Online TTS Example (Synchronous Wrapper)
    def stream_tts_playsound(self, text: str):
        """Synchronous wrapper to run async TTS function in the dedicated TTS thread."""
        if self.tts_loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._async_stream_tts_playsound(text), self.tts_loop
            )
        else:
            self.ros2_node.get_logger().error(
                "TTS event loop is not running. Cannot play audio.")

    # -------------------------------------------- Online TTS
    def _run_tts_loop(self):
        """Runs the asyncio event loop in a separate thread."""
        asyncio.set_event_loop(self.tts_loop)
        self.tts_loop.run_forever()

    # -------------------------------------------- Online TTS
    async def _async_stream_tts_playsound(self, text: str):
        """Asynchronous function to stream TTS and play sound."""
        try:
            communicate = edge_tts.Communicate(
                text, voice="en-US-GuyNeural", rate="-10%")
            # Use os.path.join for paths
            self.gpt_dir = os.path.join(
                self.ros2_node.audio_dir, "edge_tts.mp3")
            self.ros2_node.get_logger().info(f"Saving audio to {self.gpt_dir}")
            await communicate.save(self.gpt_dir)  # AWAIT the save operation

            # playsound is synchronous, so it can be called directly after await
            playsound(self.gpt_dir)
            self.ros2_node.get_logger().info(
                f"Played audio from {self.gpt_dir}")

        except Exception as e:
            self.ros2_node.get_logger().error(
                f"Error in async_stream_tts_playsound: {e}")

     # -------------------------------------------- Audio Processing Functions
    def get_latest_recording(self) -> str or None:
        """
        Finds the latest .webm recording file in the specified directory.
        """
        list_of_files = glob.glob(os.path.join(
            self.ros2_node.audio_dir, 'recording_*.webm'))
        if not list_of_files:
            return None
        latest_file = max(list_of_files, key=os.path.getctime)
        return latest_file

     # -------------------------------------------- Audio Processing Functions
    def transcribe_audio(self, audio_path: str) -> str or None:
        """
        Transcribes the given audio file (supports .webm) to text using Google Speech Recognition.
        Converts .webm to .wav if necessary.
        """
        r = sr.Recognizer()
        temp_wav_path = None
        try:
            # Check if the file is .webm and convert to .wav if needed
            if audio_path.endswith('.webm'):
                self.ros2_node.get_logger().info(
                    f"Converting {audio_path} to WAV for transcription.")
                audio = AudioSegment.from_file(audio_path, format="webm")
                temp_wav_path = audio_path.replace(".webm", ".wav")
                audio.export(temp_wav_path, format="wav")
                audio_to_process = sr.AudioFile(temp_wav_path)
            elif audio_path.endswith('.wav'):
                audio_to_process = sr.AudioFile(audio_path)
            else:
                self.ros2_node.get_logger().error(
                    f"Unsupported audio format for transcription: {audio_path}")
                return None

            with audio_to_process as source:
                audio_data = r.record(source)
            text = r.recognize_google(audio_data)
            return text
        except sr.UnknownValueError:
            self.ros2_node.get_logger().error(
                "Google Speech Recognition could not understand audio")
            return None
        except sr.RequestError as e:
            self.ros2_node.get_logger().error(
                f"Could not request results from Google Speech Recognition service; {e}")
            return None
        except Exception as e:
            self.ros2_node.get_logger().error(
                f"Error during audio transcription: {e}")
            return None
        finally:
            if temp_wav_path and os.path.exists(temp_wav_path):
                os.remove(temp_wav_path)
                self.ros2_node.get_logger().info(
                    f"Removed temporary WAV file: {temp_wav_path}")
                
    # -------------------------------------------- Gemini Movement Example
    def move_robot(self, x: float, y: float) -> str:
        self.ros2_node.get_logger().info(f"Moving robot to ({x}, {y})")
        return "Moved to ({x}, {y})"