package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.jni.OrchestraJNI;
import java.util.Collection;

// sorry ctre don't sue us please
/**
 * Orchestra is used to play music through devices. It uses a "Chirp" (.chrp) music file that can be
 * generated using Phoenix Tuner. Chirp files are generated from standard MIDI files.
 *
 * <p>Any Chirp file located in the src/main/deploy directory of your FRC project will automatically
 * be copied to the roboRIO on code deploy.
 *
 * <p>Unless {@link AudioConfigs#AllowMusicDurDisable} is enabled, the robot must be enabled to play
 * music. Additionally, devices playing in Orchestra will not run any other control requests while
 * Orchestra is running. Users can {@link #pause} or {@link #stop} the Orchestra to re-enable device
 * control.
 *
 * <p>Each device can only play a single track within the music file. For multi-track files,
 * multiple devices are needed. Devices can be added with an explicit track number. Otherwise, the
 * first track will be played through the first Talon FX added, the second track will be played
 * through the second Talon FX added, etc.
 *
 * <p>To use Orchestra:
 *
 * <ul>
 *   <li>Add the Talon FXs to be used as instruments using {@link #addInstrument}.
 *   <li>Load the Chirp file to be played using {@link #loadMusic}. This can also be done in the
 *       Orchestra constructor.
 * </ul>
 *
 * Both of these can also be done in the Orchestra constructor.
 *
 * <p>Once ready, the Orchestra can be controlled using {@link #play}/{@link #pause}/{@link #stop}.
 * New music files can be loaded at any time.
 */
public class zOrchestra {
  private final OrchestraJNI jni = new OrchestraJNI();

  /** Constructor for a new Orchestra. */
  public zOrchestra() {
    jni.JNI_Create();
  }

  /**
   * Constructor for a new Orchestra using the given Chirp file.
   *
   * <p>This API is blocking on the file read.
   *
   * @param filepath The path to the music file to immediately load into the orchestra.
   */
  public zOrchestra(String filepath) {
    this();
    loadMusic(filepath);
  }

  /**
   * Constructor for a new Orchestra using the given instruments.
   *
   * @param instruments A collection of devices that will be used as instruments in the orchestra.
   */
  public zOrchestra(Collection<ParentDevice> instruments) {
    this();
    for (var instrument : instruments) {
      addInstrument(instrument);
    }
  }

  /**
   * Constructor for a new Orchestra using the given instruments and Chirp file.
   *
   * <p>This API is blocking on the file read.
   *
   * @param instruments A collection of devices that will be used as instruments in the orchestra.
   * @param filepath The path to the music file to immediately load into the orchestra.
   */
  public zOrchestra(Collection<ParentDevice> instruments, String filepath) {
    this();
    for (var instrument : instruments) {
      addInstrument(instrument);
    }
    loadMusic(filepath);
  }

  /** Closes this Orchestra instance. */
  public void close() {
    jni.JNI_Close();
  }

  /**
   * Adds an instrument to the orchestra.
   *
   * <p>This adds the instrument to the next track; it does not wrap back to track 0 if all tracks
   * have been filled. To assign multiple instruments to a track, use {@link
   *
   * @param instrument The device to add to the orchestra
   * @return Status code of adding the device
   */
  public StatusCode addInstrument(ParentDevice instrument) {
    return StatusCode.valueOf(
        jni.JNI_AddDevice(instrument.getNetwork(), instrument.getDeviceHash()));
  }

  /**
   * Adds an instrument to the orchestra on the given track.
   *
   * <p>This can be used to assign multiple instruments to a track.
   *
   * @param instrument The device to add to the orchestra
   * @param trackNumber The track number the device should play, starting at 0
   * @return Status code of adding the device
   */
  public StatusCode addInstrument(DeviceIdentifier instrument, int trackNumber) {
    return StatusCode.valueOf(
        jni.JNI_AddDeviceWithTrack(
            instrument.getNetwork(), instrument.getDeviceHash(), trackNumber));
  }

  /**
   * Clears all instruments in the orchestra.
   *
   * @return Status code of clearing all devices
   */
  public StatusCode clearInstruments() {
    return StatusCode.valueOf(jni.JNI_ClearDevices());
  }

  /**
   * Loads a Chirp file at the specified file path.
   *
   * <p>If the Chirp file is inside your "src/main/deploy" directory, it will be automatically
   * deployed to a default directory on the roboRIO when you deploy code. For these files, the name
   * and file extension is sufficient.
   *
   * <p>A Chirp file can be created from a MIDI file using Phoenix Tuner.
   *
   * <p>This API is blocking on the file read.
   *
   * @param filepath The path to the Chirp file
   * @return Status code of loading the Chirp file
   */
  public StatusCode loadMusic(String filepath) {
    return StatusCode.valueOf(jni.JNI_LoadMusic(filepath));
  }

  /**
   * Plays the loaded music file. If the player is paused, this will resume the orchestra.
   *
   * @return Status code of playing the orchestra
   */
  public StatusCode play() {
    return StatusCode.valueOf(jni.JNI_Play());
  }

  /**
   * Pauses the loaded music file. This saves the current position in the track so it can be resumed
   * later.
   *
   * @return Status code of pausing the orchestra
   */
  public StatusCode pause() {
    return StatusCode.valueOf(jni.JNI_Pause());
  }

  /**
   * Stops the loaded music file. This resets the current position in the track to the start.
   *
   * @return Status code of stopping the orchestra
   */
  public StatusCode stop() {
    return StatusCode.valueOf(jni.JNI_Stop());
  }

  /**
   * Gets whether the current track is actively playing.
   *
   * @return true if Orchestra is playing the music file
   */
  public boolean isPlaying() {
    return jni.JNI_IsPlaying();
  }

  /**
   * Gets the current timestamp of the music file. The timestamp will reset to zero whenever {@link
   * #loadMusic} or {@link #stop} is called.
   *
   * <p>If {@link #isPlaying} returns false, this method can be used to determine if the music is
   * stopped or paused.
   *
   * @return The current timestamp of the music file, in seconds
   */
  public double getCurrentTime() {
    return jni.JNI_GetCurrentTime();
  }
}
