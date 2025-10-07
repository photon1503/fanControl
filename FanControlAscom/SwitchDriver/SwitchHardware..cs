// TODO fill in this information for your driver, then remove this line!
//
// ASCOM Switch hardware class for FanControl
//
// Description:	 <To be completed by driver developer>
//
// Implements:	ASCOM Switch interface version: <To be completed by driver developer>
// Author:		(XXX) Your N. Here <your@email.here>
//

/* Response:
 * TEMP,OFF,24.42,41.30,1003.94,24.18,41.54,1005.15,N/A
 *
(temp,hum,press,temp,hum,press,dutycycle)
SIDE,0.0,0,0.0,0.0,0.030,0.010,0.005,0.00
BACK,-0.0,0,0.0,0.0,0.030,0.010,0.005,0.00

(rpm,duty,targetrpm,delta,kp,ki,kd,airflow)

Commands:
 *   R1#900   - Set target RPM for SIDE fan to 900
 *   R2#1200  - Set target RPM for BACK fan to 1200
 *   S1#100   - Set duty cycle for SIDE fan to 100 (disables PID)
 *   S2#150   - Set duty cycle for BACK fan to 150 (disables PID)
 *   P0.2     - Set Kp to 0.2
 *   I0.05    - Set Ki to 0.05
 *   D0.02    - Set Kd to 0.02
 *   T#ON     - Enable temperature-based back fan control
 *   T#OFF    - Disable temperature-based back fan control
 */

using ASCOM;
using ASCOM.Astrometry;
using ASCOM.Astrometry.AstroUtils;
using ASCOM.Astrometry.NOVAS;
using ASCOM.DeviceInterface;
using ASCOM.LocalServer;
using ASCOM.Utilities;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ASCOM.FanControl.Switch
{
    //
    // TODO Customise the InitialiseHardware() method with code to set up a communication path to your hardware and validate that the hardware exists
    //
    // TODO Customise the SetConnected() method with code to connect to and disconnect from your hardware
    // NOTE You should not need to customise the code in the Connecting, Connect() and Disconnect() members as these are already fully implemented and call SetConnected() when appropriate.
    //
    // TODO Replace the not implemented exceptions with code to implement the functions or throw the appropriate ASCOM exceptions.
    //

    /// <summary>
    /// ASCOM Switch hardware class for FanControl.
    /// </summary>
    [HardwareClass()] // Class attribute flag this as a device hardware class that needs to be disposed by the local server when it exits.
    internal static class SwitchHardware
    {
        // Constants used for Profile persistence
        internal const string comPortProfileName = "COM Port";

        internal const string comPortDefault = "COM1";
        internal const string traceStateProfileName = "Trace Level";
        internal const string traceStateDefault = "true";

        private static string DriverProgId = ""; // ASCOM DeviceID (COM ProgID) for this driver, the value is set by the driver's class initialiser.
        private static string DriverDescription = ""; // The value is set by the driver's class initialiser.
        internal static string comPort; // COM port name (if required)
        private static bool connectedState; // Local server's connected state
        private static bool connecting; // Completion variable for use with the Connect and Disconnect methods
        private static bool runOnce = false; // Flag to enable "one-off" activities only to run once.
        internal static Util utilities; // ASCOM Utilities object for use as required
        internal static AstroUtils astroUtilities; // ASCOM AstroUtilities object for use as required
        internal static TraceLogger tl; // Local server's trace logger object for diagnostic log with information that you specify

        private static List<Guid> uniqueIds = new List<Guid>(); // List of driver instance unique IDs

        internal static double sideRPM = 0;
        internal static long sideTargetRPM = 0;
        internal static double backRPM = 0;
        internal static long backTargetRPM = 0;
        internal static bool backAuto = false;
        internal static double sidePWM = 0;
        internal static double backPWM = 0;
        internal static double mirrorTemp = 0;
        internal static double mirrorHum = 0;
        internal static double mirrorPress = 0;
        internal static double outsideTemp = 0;
        internal static double outsideHum = 0;
        internal static double outsidePress = 0;
        internal static double kp = 0.2;
        internal static double ki = 0.05;
        internal static double kd = 0.02;

        /// <summary>
        /// Initializes a new instance of the device Hardware class.
        /// </summary>
        static SwitchHardware()
        {
            try
            {
                // Create the hardware trace logger in the static initialiser.
                // All other initialisation should go in the InitialiseHardware method.
                tl = new TraceLogger("", "FanControl.Hardware");

                // DriverProgId has to be set here because it used by ReadProfile to get the TraceState flag.
                DriverProgId = Switch.DriverProgId; // Get this device's ProgID so that it can be used to read the Profile configuration values

                // ReadProfile has to go here before anything is written to the log because it loads the TraceLogger enable / disable state.
                ReadProfile(); // Read device configuration from the ASCOM Profile store, including the trace state

                LogMessage("SwitchHardware", $"Static initialiser completed.");
            }
            catch (Exception ex)
            {
                try { LogMessage("SwitchHardware", $"Initialisation exception: {ex}"); } catch { }
                MessageBox.Show($"SwitchHardware - {ex.Message}\r\n{ex}", $"Exception creating {Switch.DriverProgId}", MessageBoxButtons.OK, MessageBoxIcon.Error);
                throw;
            }
        }

        /// <summary>
        /// Place device initialisation code here
        /// </summary>
        /// <remarks>Called every time a new instance of the driver is created.</remarks>
        internal static void InitialiseHardware()
        {
            // This method will be called every time a new ASCOM client loads your driver
            LogMessage("InitialiseHardware", $"Start.");

            // Add any code that you want to run every time a client connects to your driver here

            // Add any code that you only want to run when the first client connects in the if (runOnce == false) block below
            if (runOnce == false)
            {
                LogMessage("InitialiseHardware", $"Starting one-off initialisation.");

                DriverDescription = Switch.DriverDescription; // Get this device's Chooser description

                LogMessage("InitialiseHardware", $"ProgID: {DriverProgId}, Description: {DriverDescription}");

                connectedState = false; // Initialise connected to false
                utilities = new Util(); //Initialise ASCOM Utilities object
                astroUtilities = new AstroUtils(); // Initialise ASCOM Astronomy Utilities object

                LogMessage("InitialiseHardware", "Completed basic initialisation");

                // Add your own "one off" device initialisation here e.g. validating existence of hardware and setting up communications
                // If you are using a serial COM port you will find the COM port name selected by the user through the setup dialogue in the comPort variable.

                LogMessage("InitialiseHardware", $"One-off initialisation complete.");
                runOnce = true; // Set the flag to ensure that this code is not run again
            }
        }

        // PUBLIC COM INTERFACE ISwitchV3 IMPLEMENTATION

        #region Common properties and methods.

        /// <summary>
        /// Displays the Setup Dialogue form.
        /// If the user clicks the OK button to dismiss the form, then
        /// the new settings are saved, otherwise the old values are reloaded.
        /// THIS IS THE ONLY PLACE WHERE SHOWING USER INTERFACE IS ALLOWED!
        /// </summary>
        public static void SetupDialog()
        {
            // Don't permit the setup dialogue if already connected
            if (IsConnected)
                MessageBox.Show("Already connected, just press OK");

            using (SetupDialogForm F = new SetupDialogForm(tl))
            {
                var result = F.ShowDialog();
                if (result == DialogResult.OK)
                {
                    WriteProfile(); // Persist device configuration values to the ASCOM Profile store
                }
            }
        }

        /// <summary>Returns the list of custom action names supported by this driver.</summary>
        /// <value>An ArrayList of strings (SafeArray collection) containing the names of supported actions.</value>
        public static ArrayList SupportedActions
        {
            get
            {
                LogMessage("SupportedActions Get", "Returning empty ArrayList");
                return new ArrayList();
            }
        }

        /// <summary>Invokes the specified device-specific custom action.</summary>
        /// <param name="ActionName">A well known name agreed by interested parties that represents the action to be carried out.</param>
        /// <param name="ActionParameters">List of required parameters or an <see cref="String.Empty">Empty String</see> if none are required.</param>
        /// <returns>A string response. The meaning of returned strings is set by the driver author.
        /// <para>Suppose filter wheels start to appear with automatic wheel changers; new actions could be <c>QueryWheels</c> and <c>SelectWheel</c>. The former returning a formatted list
        /// of wheel names and the second taking a wheel name and making the change, returning appropriate values to indicate success or failure.</para>
        /// </returns>
        public static string Action(string actionName, string actionParameters)
        {
            LogMessage("Action", $"Action {actionName}, parameters {actionParameters} is not implemented");
            throw new ActionNotImplementedException("Action " + actionName + " is not implemented by this driver");
        }

        /// <summary>
        /// Transmits an arbitrary string to the device and does not wait for a response.
        /// Optionally, protocol framing characters may be added to the string before transmission.
        /// </summary>
        /// <param name="Command">The literal command string to be transmitted.</param>
        /// <param name="Raw">
        /// if set to <c>true</c> the string is transmitted 'as-is'.
        /// If set to <c>false</c> then protocol framing characters may be added prior to transmission.
        /// </param>
        public static void CommandBlind(string command, bool raw)
        {
            CheckConnected("CommandBlind");
            // TODO The optional CommandBlind method should either be implemented OR throw a MethodNotImplementedException
            // If implemented, CommandBlind must send the supplied command to the mount and return immediately without waiting for a response

            throw new MethodNotImplementedException($"CommandBlind - Command:{command}, Raw: {raw}.");
        }

        /// <summary>
        /// Transmits an arbitrary string to the device and waits for a boolean response.
        /// Optionally, protocol framing characters may be added to the string before transmission.
        /// </summary>
        /// <param name="Command">The literal command string to be transmitted.</param>
        /// <param name="Raw">
        /// if set to <c>true</c> the string is transmitted 'as-is'.
        /// If set to <c>false</c> then protocol framing characters may be added prior to transmission.
        /// </param>
        /// <returns>
        /// Returns the interpreted boolean response received from the device.
        /// </returns>
        public static bool CommandBool(string command, bool raw)
        {
            CheckConnected("CommandBool");
            // TODO The optional CommandBool method should either be implemented OR throw a MethodNotImplementedException
            // If implemented, CommandBool must send the supplied command to the mount, wait for a response and parse this to return a True or False value

            throw new MethodNotImplementedException($"CommandBool - Command:{command}, Raw: {raw}.");
        }

        /// <summary>
        /// Transmits an arbitrary string to the device and waits for a string response.
        /// Optionally, protocol framing characters may be added to the string before transmission.
        /// </summary>
        /// <param name="Command">The literal command string to be transmitted.</param>
        /// <param name="Raw">
        /// if set to <c>true</c> the string is transmitted 'as-is'.
        /// If set to <c>false</c> then protocol framing characters may be added prior to transmission.
        /// </param>
        /// <returns>
        /// Returns the string response received from the device.
        /// </returns>
        public static string CommandString(string command, bool raw)
        {
            CheckConnected("CommandString");
            // TODO The optional CommandString method should either be implemented OR throw a MethodNotImplementedException
            // If implemented, CommandString must send the supplied command to the mount and wait for a response before returning this to the client

            throw new MethodNotImplementedException($"CommandString - Command:{command}, Raw: {raw}.");
        }

        /// <summary>
        /// Deterministically release both managed and unmanaged resources that are used by this class.
        /// </summary>
        /// <remarks>
        /// TODO: Release any managed or unmanaged resources that are used in this class.
        ///
        /// Do not call this method from the Dispose method in your driver class.
        ///
        /// This is because this hardware class is decorated with the <see cref="HardwareClassAttribute"/> attribute and this Dispose() method will be called
        /// automatically by the  local server executable when it is irretrievably shutting down. This gives you the opportunity to release managed and unmanaged
        /// resources in a timely fashion and avoid any time delay between local server close down and garbage collection by the .NET runtime.
        ///
        /// For the same reason, do not call the SharedResources.Dispose() method from this method. Any resources used in the static shared resources class
        /// itself should be released in the SharedResources.Dispose() method as usual. The SharedResources.Dispose() method will be called automatically
        /// by the local server just before it shuts down.
        ///
        /// </remarks>
        public static void Dispose()
        {
            try { LogMessage("Dispose", $"Disposing of assets and closing down."); } catch { }

            try
            {
                // Clean up the trace logger and utility objects
                tl.Enabled = false;
                tl.Dispose();
                tl = null;
            }
            catch { }

            try
            {
                utilities.Dispose();
                utilities = null;
            }
            catch { }

            try
            {
                astroUtilities.Dispose();
                astroUtilities = null;
            }
            catch { }
        }

        /// <summary>
        /// Connect to the hardware if not already connected
        /// </summary>
        /// <param name="uniqueId">Unique ID identifying the calling driver instance.</param>
        /// <remarks>
        /// The unique ID is stored to record that the driver instance is connected and to ensure that multiple calls from the same driver are ignored.
        /// If this is the first driver instance to connect, the physical hardware link to the device is established
        /// </remarks>
        public static void Connect(Guid uniqueId)
        {
            LogMessage("Connect", $"Device instance unique ID: {uniqueId}");

            // Check whether this driver instance has already connected
            if (uniqueIds.Contains(uniqueId)) // Instance already connected
            {
                // Ignore the request, the unique ID is already in the list
                LogMessage("Connect", $"Ignoring request to connect because the device is already connected.");

                return;
            }

            // Set the connection in progress flag
            connecting = true;

            // Driver instance not yet connected, so start a task to connect to the device hardware and return while the task runs in the background
            // Discard the returned task value because this a "fire and forget" task
            LogMessage("Connect", $"Starting Connect task...");
            _ = Task.Run(() =>
            {
                try
                {
                    // Set the Connected state to true, waiting until it completes
                    LogMessage("ConnectTask", $"Setting connection state to true");
                    SetConnected(uniqueId, true);
                    LogMessage("ConnectTask", $"Connected set true");
                }
                catch (Exception ex)
                {
                    LogMessage("ConnectTask", $"Exception - {ex.Message}\r\n{ex}");
                    throw;
                }
                finally
                {
                    connecting = false;
                    LogMessage("ConnectTask", $"Connecting set false");
                }
            });
            LogMessage("Connect", $"Connect task started OK");
        }

        /// <summary>
        /// Disconnect from the device asynchronously using Connecting as the completion variable
        /// </summary>
        /// <param name="uniqueId">Unique ID identifying the calling driver instance.</param>
        /// <remarks>
        /// The list of connected driver instance IDs is queried to determine whether this driver instance is connected and, if so, it is removed from the connection list.
        /// The unique ID ensures that multiple calls from the same driver are ignored.
        /// If this is the last connected driver instance, the physical link to the device hardware is disconnected.
        /// </remarks>
        public static void Disconnect(Guid uniqueId)
        {
            LogMessage("Disconnect", $"Device instance unique ID: {uniqueId}");

            // Check whether this driver instance has already disconnected
            if (!uniqueIds.Contains(uniqueId)) // Instance already disconnected
            {
                // Ignore the request, the unique ID is already removed from the list
                LogMessage("Disconnect", $"Ignoring request to disconnect because the device is already disconnected.");
                return;
            }

            // Set the Disconnect in progress flag
            connecting = true;

            // Start a task to disconnect from the device hardware and return while the task runs in the background
            // Discard the returned task value because this a "fire and forget" task
            LogMessage("Disconnect", $"Starting Disconnect task...");
            _ = Task.Run(() =>
            {
                try
                {
                    // Set the Connected state to false, waiting until it completes
                    LogMessage("DisconnectTask", $"Setting connection state to false");
                    SetConnected(uniqueId, false);
                    LogMessage("DisconnectTask", $"Connected set false");
                }
                catch (Exception ex)
                {
                    LogMessage("DisconnectTask", $"Exception - {ex.Message}\r\n{ex}");
                    throw;
                }
                finally
                {
                    connecting = false;
                    LogMessage("DisconnectTask", $"Connecting set false");
                }
            });
            LogMessage("Disconnect", $"Disconnect task started OK");
        }

        /// <summary>
        /// Completion variable for the asynchronous Connect() and Disconnect()  methods
        /// </summary>
        public static bool Connecting
        {
            get
            {
                return connecting;
            }
        }

        /// <summary>
        /// Synchronously connect to or disconnect from the hardware
        /// </summary>
        /// <param name="uniqueId">Driver's unique ID</param>
        /// <param name="newState">New state: Connected or Disconnected</param>
        public static void SetConnected(Guid uniqueId, bool newState)
        {
            // Check whether we are connecting or disconnecting
            if (newState) // We are connecting
            {
                // Check whether this driver instance has already connected
                if (uniqueIds.Contains(uniqueId)) // Instance already connected
                {
                    // Ignore the request, the unique ID is already in the list
                    LogMessage("SetConnected", $"Ignoring request to connect because the device is already connected.");
                }
                else // Instance not already connected, so connect it
                {
                    // Check whether this is the first connection to the hardware
                    if (uniqueIds.Count == 0) // This is the first connection to the hardware so initiate the hardware connection
                    {
                        //
                        // Add hardware connect logic here
                        SharedResources.SharedSerial.PortName = comPort; // Set the COM port
                        SharedResources.SharedSerial.Speed = SerialSpeed.ps115200; // Set the COM port speed
                        SharedResources.SharedSerial.Connected = true; // Connect to the COM port
                                                                       //   connectedState = true;

                        // start a timer to read serial using DataReceived function every second
                        double interval = 1000; // milliseconds
                        System.Timers.Timer timer = new System.Timers.Timer(interval);
                        timer.Elapsed += new System.Timers.ElapsedEventHandler(DataReceived);
                        timer.Start();

                        //
                        LogMessage("SetConnected", $"Connecting to hardware.");
                    }
                    else // Other device instances are connected so the hardware is already connected
                    {
                        // Since the hardware is already connected no action is required
                        LogMessage("SetConnected", $"Hardware already connected.");
                    }

                    // The hardware either "already was" or "is now" connected, so add the driver unique ID to the connected list
                    uniqueIds.Add(uniqueId);
                    LogMessage("SetConnected", $"Unique id {uniqueId} added to the connection list.");
                }
            }
            else // We are disconnecting
            {
                // Check whether this driver instance has already disconnected
                if (!uniqueIds.Contains(uniqueId)) // Instance not connected so ignore request
                {
                    // Ignore the request, the unique ID is not in the list
                    LogMessage("SetConnected", $"Ignoring request to disconnect because the device is already disconnected.");
                }
                else // Instance currently connected so disconnect it
                {
                    // Remove the driver unique ID to the connected list
                    uniqueIds.Remove(uniqueId);
                    LogMessage("SetConnected", $"Unique id {uniqueId} removed from the connection list.");

                    // Check whether there are now any connected driver instances
                    if (uniqueIds.Count == 0) // There are no connected driver instances so disconnect from the hardware
                    {
                        //
                        SharedResources.SharedSerial.Connected = false; // Disconnect from the COM port
                                                                        //  connectedState = false;

                        //
                    }
                    else // Other device instances are connected so do not disconnect the hardware
                    {
                        // No action is required
                        LogMessage("SetConnected", $"Hardware already connected.");
                    }
                }
            }

            // Log the current connected state
            LogMessage("SetConnected", $"Currently connected driver ids:");
            foreach (Guid id in uniqueIds)
            {
                LogMessage("SetConnected", $" ID {id} is connected");
            }
        }

        /// <summary>
        /// Returns a description of the device, such as manufacturer and model number. Any ASCII characters may be used.
        /// </summary>
        /// <value>The description.</value>
        public static string Description
        {
            // TODO customise this device description if required
            get
            {
                LogMessage("Description Get", DriverDescription);
                return DriverDescription;
            }
        }

        /// <summary>
        /// Descriptive and version information about this ASCOM driver.
        /// </summary>
        public static string DriverInfo
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                // TODO customise this driver description if required
                string driverInfo = $"Information about the driver itself. Version: {version.Major}.{version.Minor}";
                LogMessage("DriverInfo Get", driverInfo);
                return driverInfo;
            }
        }

        /// <summary>
        /// A string containing only the major and minor version of the driver formatted as 'm.n'.
        /// </summary>
        public static string DriverVersion
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverVersion = $"{version.Major}.{version.Minor}";
                LogMessage("DriverVersion Get", driverVersion);
                return driverVersion;
            }
        }

        /// <summary>
        /// The interface version number that this device supports.
        /// </summary>
        public static short InterfaceVersion
        {
            // set by the driver wizard
            get
            {
                LogMessage("InterfaceVersion Get", "3");
                return Convert.ToInt16("3");
            }
        }

        /// <summary>
        /// The short name of the driver, for display purposes
        /// </summary>
        public static string Name
        {
            // TODO customise this device name as required
            get
            {
                string name = "Fan Control";
                LogMessage("Name Get", name);
                return name;
            }
        }

        #endregion Common properties and methods.

        #region ISwitch Implementation

        private static short numSwitch = 11;

        /// <summary>
        /// The number of switches managed by this driver
        /// </summary>
        /// <returns>The number of devices managed by this driver.</returns>
        internal static short MaxSwitch
        {
            get
            {
                LogMessage("MaxSwitch Get", numSwitch.ToString());
                return numSwitch;
            }
        }

        /// <summary>
        /// Return the name of switch device n.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>The name of the device</returns>
        internal static string GetSwitchName(short id)
        {
            Validate("GetSwitchName", id);
            switch (id)
            {
                case 0: return "Side PWM";
                case 1: return "Side RPM Target";
                case 2: return "Back PWM";
                case 3: return "Back RPM Target";
                case 4: return "Back Auto";
                case 5: return "Mirror Temp";
                case 6: return "Mirror Hum";
                case 7: return "Mirror Press";
                case 8: return "Outside Temp";
                case 9: return "Outside Hum";
                case 10: return "Outside Press";
                default:
                    LogMessage("GetSwitchName", $"GetSwitchName({id}) - Invalid switch ID");
                    throw new ASCOM.InvalidValueException($"GetSwitchName({id}) - Invalid switch ID");
            }
        }

        /// <summary>
        /// Set a switch device name to a specified value.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <param name="name">The name of the device</param>
        internal static void SetSwitchName(short id, string name)
        {
            Validate("SetSwitchName", id);
            LogMessage("SetSwitchName", $"SetSwitchName({id}) = {name} - not implemented");
            throw new MethodNotImplementedException("SetSwitchName");
        }

        /// <summary>
        /// Gets the description of the specified switch device. This is to allow a fuller description of
        /// the device to be returned, for example for a tool tip.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>
        /// String giving the device description.
        /// </returns>
        internal static string GetSwitchDescription(short id)
        {
            Validate("GetSwitchDescription", id);
            switch (id)
            {
                case 0: return "Side fan PWM value (0-255)";
                case 1: return "Side fan target RPM (0 for off)";
                case 2: return "Back fan PWM value (0-255)";
                case 3: return "Back fan target RPM (0 for off)";
                case 4: return "Back fan auto mode (true/false)";
                case 5: return "Mirror temperature (C)";
                case 6: return "Mirror humidity (%)";
                case 7: return "Mirror pressure (hPa)";
                case 8: return "Outside temperature (C)";
                case 9: return "Outside humidity (%)";
                case 10: return "Outside pressure (hPa)";
                default:
                    LogMessage("GetSwitchDescription", $"GetSwitchDescription({id}) - Invalid switch ID");
                    throw new ASCOM.InvalidValueException($"GetSwitchDescription({id}) - Invalid switch ID");
            }
        }

        /// <summary>
        /// Reports if the specified switch device can be written to, default true.
        /// This is false if the device cannot be written to, for example a limit switch or a sensor.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>
        /// <c>true</c> if the device can be written to, otherwise <c>false</c>.
        /// </returns>
        internal static bool CanWrite(short id)
        {
            bool writable = true;
            Validate("CanWrite", id);
            switch (id)
            {
                case 0: // Side PWM
                case 1: // Side RPM Target
                case 2: // Back PWM
                case 3: // Back RPM Target
                case 4: // Back Auto
                    writable = true;
                    break;

                case 5: // Mirror Temp
                case 6: // Mirror Hum
                case 7: // Mirror Press
                case 8: // Outside Temp
                case 9: // Outside Hum
                case 10: // Outside Press
                    writable = false;
                    break;

                default:
                    LogMessage("CanWrite", $"CanWrite({id}) - Invalid switch ID");
                    throw new ASCOM.InvalidValueException($"CanWrite({id}) - Invalid switch ID");
            }
            return true;
        }

        #region Boolean switch members

        /// <summary>
        /// Return the state of switch device id as a boolean
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>True or false</returns>
        internal static bool GetSwitch(short id)
        {
            Validate("GetSwitch", id);
            //TODO
            return false;
        }

        /// <summary>
        /// Sets a switch controller device to the specified state, true or false.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <param name="state">The required control state</param>
        internal static void SetSwitch(short id, bool state)
        {
            Validate("SetSwitch", id);
            if (!CanWrite(id))
            {
                var str = $"SetSwitch({id}) - Cannot Write";
                LogMessage("SetSwitch", str);
                throw new MethodNotImplementedException(str);
            }
            LogMessage("SetSwitch", $"SetSwitch({id}) = {state} - not implemented");
            throw new MethodNotImplementedException("SetSwitch");
        }

        #endregion Boolean switch members

        #region Analogue members

        /// <summary>
        /// Returns the maximum value for this switch device, this must be greater than <see cref="MinSwitchValue"/>.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>The maximum value to which this device can be set or which a read only sensor will return.</returns>
        internal static double MaxSwitchValue(short id)
        {
            Validate("MaxSwitchValue", id);
            switch (id)
            {
                case 0: return 255; // Side PWM
                case 1: return 2200; // Side RPM Target
                case 2: return 255; // Back PWM

                case 3: return 2200; // Back RPM Target
                case 4: return 1; // Back Auto
                case 5: return 100; // Mirror Temp
                case 6: return 100; // Mirror Hum
                case 7: return 1100; // Mirror Press
                case 8: return 40; // Outside Temp
                case 9: return 100; // Outside Hum
                case 10: return 1100; // Outside Press
            }
            return 0;
        }

        /// <summary>
        /// Returns the minimum value for this switch device, this must be less than <see cref="MaxSwitchValue"/>
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>The minimum value to which this device can be set or which a read only sensor will return.</returns>
        internal static double MinSwitchValue(short id)
        {
            Validate("MinSwitchValue", id);
            return 0;
        }

        /// <summary>
        /// Returns the step size that this device supports (the difference between successive values of the device).
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>The step size for this device.</returns>
        internal static double SwitchStep(short id)
        {
            Validate("SwitchStep", id);
            switch (id)
            {
                case 0: return 1; // Side PWM
                case 1: return 1; // Side RPM Target
                case 2: return 1; // Back PWM
                case 3: return 1; // Back RPM Target
                case 4: return 1; // Back Auto
                case 5: return 0.1; // Mirror Temp
                case 6: return 0.1; // Mirror Hum
                case 7: return 0.1; // Mirror Press
                case 8: return 0.1; // Outside Temp
                case 9: return 0.1; // Outside Hum
                case 10: return 0.1; // Outside Press
            }
            return 0;
        }

        /// <summary>
        /// Returns the value for switch device id as a double
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>The value for this switch, this is expected to be between <see cref="MinSwitchValue"/> and
        /// <see cref="MaxSwitchValue"/>.</returns>
        internal static double GetSwitchValue(short id)
        {
            Validate("GetSwitchValue", id);
            switch (id)
            {
                case 0: return sidePWM;
                case 1: return sideRPM;
                case 2: return backPWM;
                case 3: return backRPM;
                case 4: return backAuto ? 1 : 0;
                case 5: return mirrorTemp;
                case 6: return mirrorHum;
                case 7: return mirrorPress;
                case 8: return outsideTemp;
                case 9: return outsideHum;
                case 10: return outsidePress;
                default:
                    return 0;
            }
        }

        /// <summary>
        /// Set the value for this device as a double.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <param name="value">The value to be set, between <see cref="MinSwitchValue"/> and <see cref="MaxSwitchValue"/></param>
        internal static void SetSwitchValue(short id, double value)
        {
            Validate("SetSwitchValue", id, value);
            if (!CanWrite(id))
            {
                LogMessage("SetSwitchValue", $"SetSwitchValue({id}) - Cannot write");
                throw new ASCOM.MethodNotImplementedException($"SetSwitchValue({id}) - Cannot write");
            }
            LogMessage("SetSwitchValue", $"SetSwitchValue({id}) = {value} - not implemented");
            throw new MethodNotImplementedException("SetSwitchValue");
        }

        #endregion Analogue members

        #region Async members

        /// <summary>
        /// Set a boolean switch's state asynchronously
        /// </summary>
        /// <exception cref="MethodNotImplementedException">When CanAsync(id) is false.</exception>
        /// <param name="id">Switch number.</param>
        /// <param name="state">New boolean state.</param>
        /// <remarks>
        /// <p style="color:red"><b>This is an optional method and can throw a <see cref="MethodNotImplementedException"/> when <see cref="CanAsync(short)"/> is <see langword="false"/>.</b></p>
        /// </remarks>
        public static void SetAsync(short id, bool state)
        {
            Validate("SetAsync", id);
            if (!CanAsync(id))
            {
                var message = $"SetAsync({id}) - Switch cannot operate asynchronously";
                LogMessage("SetAsync", message);
                throw new MethodNotImplementedException(message);
            }

            // Implement async support here if required
            LogMessage("SetAsync", $"SetAsync({id}) = {state} - not implemented");
            throw new MethodNotImplementedException("SetAsync");
        }

        /// <summary>
        /// Set a switch's value asynchronously
        /// </summary>
        /// <param name="id">Switch number.</param>
        /// <param name="value">New double value.</param>
        /// <p style="color:red"><b>This is an optional method and can throw a <see cref="MethodNotImplementedException"/> when <see cref="CanAsync(short)"/> is <see langword="false"/>.</b></p>
        /// <exception cref="MethodNotImplementedException">When CanAsync(id) is false.</exception>
        /// <remarks>
        /// <p style="color:red"><b>This is an optional method and can throw a <see cref="MethodNotImplementedException"/> when <see cref="CanAsync(short)"/> is <see langword="false"/>.</b></p>
        /// </remarks>
        public static void SetAsyncValue(short id, double value)
        {
            Validate("SetSwitchValue", id, value);
            if (!CanWrite(id))
            {
                LogMessage("SetSwitchValue", $"SetSwitchValue({id}) - Cannot write");
                throw new ASCOM.MethodNotImplementedException($"SetSwitchValue({id}) - Cannot write");
            }

            // Implement async support here if required
            LogMessage("SetSwitchValue", $"SetSwitchValue({id}) = {value} - not implemented");
            throw new MethodNotImplementedException("SetSwitchValue");
        }

        /// <summary>
        /// Flag indicating whether this switch can operate asynchronously.
        /// </summary>
        /// <param name="id">Switch number.</param>
        /// <returns>True if the switch can operate asynchronously.</returns>
        /// <exception cref="MethodNotImplementedException">When CanAsync(id) is false.</exception>
        /// <remarks>
        /// <p style="color:red"><b>This is a mandatory method and must not throw a <see cref="MethodNotImplementedException"/>.</b></p>
        /// </remarks>
        public static bool CanAsync(short id)
        {
            const bool ASYNC_SUPPORT_DEFAULT = false;

            Validate("CanAsync", id);

            // Default behaviour is not to support async operation
            LogMessage("CanAsync", $"CanAsync({id}): {ASYNC_SUPPORT_DEFAULT}");
            return ASYNC_SUPPORT_DEFAULT;
        }

        /// <summary>
        /// Completion variable for asynchronous switch state change operations.
        /// </summary>
        /// <param name="id">Switch number.</param>
        /// <exception cref="OperationCancelledException">When an in-progress operation is cancelled by the <see cref="CancelAsync(short)"/> method.</exception>
        /// <returns>False while an asynchronous operation is underway and true when it has completed.</returns>
        /// <remarks>
        /// <p style="color:red"><b>This is a mandatory method and must not throw a <see cref="MethodNotImplementedException"/>.</b></p>
        /// </remarks>
        public static bool StateChangeComplete(short id)
        {
            const bool STATE_CHANGE_COMPLETE_DEFAULT = true;

            Validate("StateChangeComplete", id);
            LogMessage("StateChangeComplete", $"StateChangeComplete({id}) - Returning {STATE_CHANGE_COMPLETE_DEFAULT}");
            return STATE_CHANGE_COMPLETE_DEFAULT;
        }

        /// <summary>
        /// Cancels an in-progress asynchronous state change operation.
        /// </summary>
        /// <param name="id">Switch number.</param>
        /// <exception cref="MethodNotImplementedException">When it is not possible to cancel an asynchronous change.</exception>
        /// <remarks>
        /// <p style="color:red"><b>This is an optional method and can throw a <see cref="MethodNotImplementedException"/>.</b></p>
        /// This method must be implemented if it is possible for the device to cancel an asynchronous state change operation, otherwise it must throw a <see cref="MethodNotImplementedException"/>.
        /// </remarks>
        public static void CancelAsync(short id)
        {
            Validate("CancelAsync", id);
            LogMessage("CancelAsync", $"CancelAsync({id}) - not implemented");
            throw new MethodNotImplementedException("CancelAsync");
        }

        #endregion Async members

        #endregion ISwitch Implementation

        #region Private methods

        /// <summary>
        /// Checks that the switch id is in range and throws an InvalidValueException if it isn't
        /// </summary>
        /// <param name="message">The message.</param>
        /// <param name="id">The id.</param>
        private static void Validate(string message, short id)
        {
            if (id < 0 || id >= numSwitch)
            {
                LogMessage(message, string.Format("Switch {0} not available, range is 0 to {1}", id, numSwitch - 1));
                throw new InvalidValueException(message, id.ToString(), string.Format("0 to {0}", numSwitch - 1));
            }
        }

        /// <summary>
        /// Checks that the switch id and value are in range and throws an
        /// InvalidValueException if they are not.
        /// </summary>
        /// <param name="message">The message.</param>
        /// <param name="id">The id.</param>
        /// <param name="value">The value.</param>
        private static void Validate(string message, short id, double value)
        {
            Validate(message, id);
            var min = MinSwitchValue(id);
            var max = MaxSwitchValue(id);
            if (value < min || value > max)
            {
                LogMessage(message, string.Format("Value {1} for Switch {0} is out of the allowed range {2} to {3}", id, value, min, max));
                throw new InvalidValueException(message, value.ToString(), string.Format("Switch({0}) range {1} to {2}", id, min, max));
            }
        }

        #endregion Private methods

        #region Private properties and methods

        // Useful methods that can be used as required to help with driver development

        /// <summary>
        /// Returns true if there is a valid connection to the driver hardware
        /// </summary>
        private static bool IsConnected
        {
            get
            {
                // TODO check that the driver hardware connection exists and is connected to the hardware
                return connectedState;
            }
        }

        /// <summary>
        /// Use this function to throw an exception if we aren't connected to the hardware
        /// </summary>
        /// <param name="message"></param>
        private static void CheckConnected(string message)
        {
            if (!IsConnected)
            {
                throw new NotConnectedException(message);
            }
        }

        /// <summary>
        /// Read the device configuration from the ASCOM Profile store
        /// </summary>
        internal static void ReadProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Switch";
                tl.Enabled = Convert.ToBoolean(driverProfile.GetValue(DriverProgId, traceStateProfileName, string.Empty, traceStateDefault));
                comPort = driverProfile.GetValue(DriverProgId, comPortProfileName, string.Empty, comPortDefault);
            }
        }

        /// <summary>
        /// Write the device configuration to the  ASCOM  Profile store
        /// </summary>
        internal static void WriteProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Switch";
                driverProfile.WriteValue(DriverProgId, traceStateProfileName, tl.Enabled.ToString());
                driverProfile.WriteValue(DriverProgId, comPortProfileName, comPort.ToString());
            }
        }

        /// <summary>
        /// Log helper function that takes identifier and message strings
        /// </summary>
        /// <param name="identifier"></param>
        /// <param name="message"></param>
        internal static void LogMessage(string identifier, string message)
        {
            tl.LogMessageCrLf(identifier, message);
        }

        /// <summary>
        /// Log helper function that takes formatted strings and arguments
        /// </summary>
        /// <param name="identifier"></param>
        /// <param name="message"></param>
        /// <param name="args"></param>
        internal static void LogMessage(string identifier, string message, params object[] args)
        {
            var msg = string.Format(message, args);
            LogMessage(identifier, msg);
        }

        // read incoming messages from serial port
        private static void DataReceived(object source, System.Timers.ElapsedEventArgs e)
        {
            try
            {
                // [0D][0A]
                string msg = SharedResources.SharedSerial.ReceiveTerminated("\n");

                if (string.IsNullOrEmpty(msg))
                {
                    return;
                }

                tl.LogMessage("DataReceived", $"Received message: {msg}");
                /*  * TEMP,OFF,24.42,41.30,1003.94,24.18,41.54,1005.15,N/A
  *
 (temp,hum,press,temp,hum,press,dutycycle)
 SIDE,0.0,0,0.0,0.0,0.030,0.010,0.005,0.00
 BACK,-0.0,0,0.0,0.0,0.030,0.010,0.005,0.00

 (rpm,duty,targetrpm,delta,kp,ki,kd,airflow)
 */
                // split message
                string[] parts = msg.Split(',');
                if (parts.Length > 1)
                {
                    switch (parts[0])
                    {
                        case "SIDE":
                            if (parts.Length >= 4)
                            {
                                double.TryParse(parts[1], out double tsideRpm);
                                double.TryParse(parts[2], out double tsideDuty);
                                long.TryParse(parts[3], out long tsideTarget);

                                sideRPM = tsideRpm;
                                sidePWM = tsideDuty;
                                sideTargetRPM = tsideTarget;
                            }
                            break;

                        case "BACK":
                            if (parts.Length >= 4)
                            {
                                double.TryParse(parts[1], out double tbackRpm);
                                double.TryParse(parts[2], out double tbackDuty);
                                long.TryParse(parts[3], out long tbackTarget);

                                backRPM = tbackRpm;
                                backPWM = tbackDuty;
                                backTargetRPM = tbackTarget;
                            }
                            break;

                        case "TEMP":
                            if (parts.Length >= 7)
                            {
                                double.TryParse(parts[2], out double tmirrorTemp);
                                double.TryParse(parts[3], out double tmirrorHum);
                                double.TryParse(parts[4], out double tmirrorPress);
                                double.TryParse(parts[5], out double toutsideTemp);
                                double.TryParse(parts[6], out double toutsideHum);
                                double.TryParse(parts[7], out double toutsidePress);

                                mirrorHum = tmirrorHum;
                                mirrorTemp = tmirrorTemp;
                                mirrorPress = tmirrorPress;
                                outsideTemp = toutsideTemp;
                                outsideHum = toutsideHum;
                                outsidePress = toutsidePress;
                            }
                            break;
                    }
                }
            }
            catch (Exception ex)
            {
                LogMessage("DataReceived", $"Exception reading serial data: {ex.Message}\r\n{ex}");
            }
        }

        #endregion Private properties and methods
    }
}