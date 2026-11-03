#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise ImportError('This module only supports Python 3.4 or later. Try use `python3`')

from enum import Enum, IntEnum
import serial, glob
import time, re
import serial.tools.list_ports

class Bad_Command_Parameter_or_Timing(Exception):
    '''Raised if the command, parameter, or timing is wrong.
    
    This exception raises when the controller responds 'NG'.
    It offen occurs when drive commands are sent to BUSY controller.
    '''
    pass
class Not_Supported(Exception):
    ''' Raised if the command is not supported by the controller.
    '''
    def __init__(self, message=''):
        if message: print(message)

class Undefined_Controller(Exception):
    ''' Raised if the controller is not defined or implemented. '''
    pass

class Undefined_Stage(Exception):
    ''' Raised if the stage is not defined or implemented. '''
    pass

class Controllers(Enum):
    ''' Stage controller's type.
    
    Attributes
    ----------
    SHOT_302GS
    SHOT_304GS
    SHOT_702
    
    TODO
    ----
    Other controllers such as OSM and HIT series.
    '''
    SHOT_302GS = 1
    SHOT_304GS = 2
    SHOT_702 = 3
    MAC_04 = 4
    Undefined = 99
    
    
class Controller_Modes(Enum):
    ''' Instruction set of controller's operation.
    
    Attributes
    ----------
    SHOT
        SHOT instruction set.
    SHOT_enhanced
        Enhanced SHOT instruction set.
    HIT
        HIT instruction set.
    '''
    SHOT = 0
    SHOT_enhanced = 1
    HIT = 2
    MAC = 3
    
class Comm_ack(Enum):
    ''' COMM-ACK status of the controller.
    
    Attributes
    ----------
    MAIN
    SUB
    '''
    MAIN = 0
    SUB = 1
    
class Excitation(IntEnum):
    ''' Excitation mode of the stepping moter.
    
    Attributes
    ----------
    Free
    Hold
    '''
    Free = 0
    Hold = 1

class Stages(Enum):
    ''' Stage type.
    
    Attributes
    ----------
    SGSP**_**
        SGSP series stages. e.g. Both SGSP46-500(X) and SGSP46-500(Z) are same type: SGSP46_500.
    SGSP_**YAW
    SGSP_**A**
    OSMS**-**
        Compatible stages to SGSP series.
    HST_**
    HST_**YAW
    HPS**_**
    TAMM**_**
    '''
    #
    # Linear translation stages
    # , 1 means 1 micro-meter.
    Linear_stage = (0,0)
    
    # SGSP series 
    SGSP15_10 = (151, 2)
    SGSP20_20 = (202, 2)
    SGSP20_35 = (203, 2)
    SGSP20_85 = (208, 2)
    SGSP26_50 = (265, 4)
    SGSP26_100 = (2610, 4) 
    SGSP26_150 = (2615, 4)
    SGSP26_200 = (2620, 4)
    SGSP33_50 = (335, 12)
    SGSP33_100 = (3310, 12)
    SGSP33_200 = (3320, 12)
    SGSP46_300 = (4630, 20)
    SGSP46_400 = (4640, 20)
    SGSP46_500 = (4650, 20)
    SGSP46_800 = (4680, 20)
    SGSP65_1200 = (6512, 50)
    SGSP65_1500 = (6515, 50)
    
    # OSMS series are compatible to SGSP series.
    OSMS15_10 = SGSP15_10
    OSMS20_20 = SGSP20_20
    OSMS20_35 = SGSP20_35
    OSMS20_85 = SGSP20_85
    OSMS26_50 = SGSP26_50 
    OSMS26_100 = SGSP26_100
    OSMS26_150 = SGSP26_150
    OSMS26_200 = SGSP26_200 
    OSMS33_50 = SGSP33_50
    OSMS33_100 = SGSP33_100
    OSMS33_200 = SGSP33_200
    OSMS46_300 = SGSP46_300
    OSMS46_400 = SGSP46_400
    OSMS46_500 = SGSP46_500
    OSMS46_800 = SGSP46_800
    OSMS65_1200 = SGSP65_1200
    OSMS65_1500 = SGSP65_1500 
    
    # HST series
    HST_50 = (5, 4)
    HST_100 = (10,4) 
    HST_200 = (20, 4)
    
    # HPS series
    HPS60_20 = (602, 2)
    HPS80_50 = (805, 2)
    HPS120_60 = (1206, 2) 
    
    # TAMM series
    TAMM40_10 = (401, 2)
    TAMM60_15 = (601, 2)
    TAMM100_50 = (1005, 2)
    TAMM100_100 = (1001, 2)
    
    Linear_stage_end = (9999,0)
    #
    # Rotation stages
    # ,  1 means 0.001 degree.
    Rotation_stage = (10000,0)
    
    # SGSP series
    SGSP_40YAW = (10040, 5)
    SGSP_60YAW = (10060, 5)
    SGSP_80YAW = (10080, 5)
    SGSP_120YAW = (10120, 5)
    SGSP_160YAW = (10160, 5)

    # OSMS series are compatible to SGSP series.
    OSMS_40YAW = SGSP_40YAW
    OSMS_60YAW = SGSP_60YAW
    OSMS_80YAW = SGSP_80YAW
    OSMS_120YAW = SGSP_120YAW
    OSMS_160YAW = SGSP_160YAW
    
    # HST series
    HST_120YAW = (11120, 5)
    HST_160YAW = (11160, 5)
    
    # TODO: HDS series
    
    Rotation_stage_end = (19999,0)
    #
    # Gonio stage
    #
    Gonio_stage = (20000,0)
    
    # SGSP series
    SGSP_60A75 = (26075, 2)
    SGSP_60A100 = (26010, 1)
    SGSP_60A130 = (26030, 1)
    
    Gonio_stage_end = (29999,0)
    
    Gimbal_stage = (30000,0)
    # MHDM Gimbal mirror holders series.
    MHDM_150 = (30015, 720)
    MHDM_200 = (30020, 720)
    MHDM_254 = (30025, 720)
    MHDM_300 = (30030, 720)
    
    Gimbal_stage_end = (39999,0)
    
    
    def __init__(self, uid, baserate):
        
        self.uid = uid
        self.baserate = baserate
    
    @property    
    def is_linear(self):
        ''' Returns if the stage is a linear translation stage or not.'''
        return int(self.uid) > int(Stages.Linear_stage.uid) and int(self.uid) < int(Stages.Linear_stage_end.uid)
    
    @property     
    def is_rotation(self):
        ''' Returns if the stage is a rotation stage or not.'''
        return int(self.uid) > int(Stages.Rotation_stage.uid) and int(self.uid) < int(Stages.Rotation_stage_end.uid)
    
    @property
    def is_gonio(self):
        ''' Returns if the stage is a gonio stage or not.'''
        return int(self.uid) > int(Stages.Gonio_stage.uid) and int(self.uid) < int(Stages.Gonio_stage_end.uid)
    
    
    #def get_micro_meter_per_pulse(self):
    #    if not self.is_linear_stage(): raise Undefined_Stage(stype.name + ' is not a linear stage.')
    #    return self.baserate
    #    
    #def get_milli_degree_per_pulse(self):
    #    if not (self.is_rotation() or self.is_gonio()): raise Undefined_Stage(self.name + ' is not a rotation stage.')
    #    return self.baserate
  

class Controller:
    '''Class for a controller's parameters.
    
    Parameters
    ----------
    ctype : Controllers
        Type of the controller.
        
    Raises
    ------
    Undefined_Controller
    
    Attributes
    ----------
    ctype : Controllers
        Type of a controller.
    baudrate : int
        Baudrate of RS232C communication.
    delimiter : str
        Delimiter of RS232C communication.
    comm_ack : Comm_ack
        COMM/ACK mode.
        
    '''
    def __init__(self, ctype):
        self.ctype = ctype
        self.parity = 'N'
        self.databit = 8
        self.stopbit = 1
        if ctype is Controllers.MAC_04:
            self.rtscts = False
        else:
            self.rtscts = True
        self.read_timeout = 1
        self.write_timeout = 1
        self.delimiter = b'\r\n'
        self.comm_ack = Comm_ack.MAIN
        # Load default values.
        if self.ctype is Controllers.SHOT_302GS or self.ctype is Controllers.SHOT_304GS:
            self.baudrate = 9600
            self.cmode = Controller_Modes.SHOT
        elif ctype is Controllers.SHOT_702:
            self.baudrate = 38400
            self.cmode = Controller_Modes.SHOT
        elif ctype is Controllers.MAC_04:
            self.baudrate = 115200
            self.cmode = Controller_Modes.MAC
        else:
            raise Undefined_Controller('Controller is undefined.')
    
    @property
    def __name__(self): return self.ctype.name
    
    def __str__(self):
        return (f"SigmaKoki controller '{self.__name__}', with COM parameters: "
                f"baudrate={self.baudrate}, parity='{self.parity}', databit={self.databit}, stopbit={self.stopbit}, RTS/CTS={ self.rtscts}, delimiter='{self.delimiter.decode('ascii')}'")
    
    def __is_30X(self):
        return self.ctype is Controllers.SHOT_302GS or self.ctype is Controllers.SHOT_304GS
    def __is_70X(self):
        return self.ctype is Controllers.SHOT_702
    def __is_MACX(self):
        return self.ctype is Controllers.MAC_04
    
    def is_SHOT(self):
        if self.cmode is Controller_Modes.SHOT: return True
        if self.cmode is Controller_Modes.SHOT_enhanced: return True
        return False

    def is_MAC(self): return self.cmode is Controller_Modes.MAC
        
    def is_HIT(self): return self.cmode is Controller_Modes.HIT
    
    def get_support_baudrates(self):
        ''' Get the tuple of supported baudrates of this controller.
        
        Returns
        -------
        baudrates : tuple
            A list of supported baudrates.
        '''
        if self.__is_30X(): return (4800, 9600, 19200, 38400)
        if self.__is_70X(): return (38400,)
        if self.__is_MACX(): return (115200,)
        return ()
        
    def get_support_divisions(self):
        ''' Get the tuple of supported divisions of this controller.
        
        Returns
        -------
        divisions : tuple
            A list of supportd division values.
        '''
        return (1,2,4,5,8,10,20,25,40,50,80,100,125,200,250)
    
    def get_default_division(self):
        '''Get the default microstep division of this controller'''
        return 2
    
    def get_support_axes(self):
        ''' Get the maximum number of controllable stages.
        
        This method does not check the value of AXIS memory switch nor how many stages are connected.
        
        Returns
        -------
        num : int
            The number of controllable stages.
        '''
        if self.ctype == Controllers.SHOT_304GS or self.ctype == Controllers.MAC_04:
            return 4
        else:
            return 2
        
    def get_support_speed_ranges(self):
        '''Get the range of speed.
        
        Returns
        -------
        ((S_min, S_max), (F_min, F_max), (R_min, R_max))
        '''
        return ((1, 500000), (1, 500000), (0, 1000))        
        
    # Whether the controller supports specific command.
    # Moving operations
    def is_support_A(self):
        ''' Check if the controller supports "A" command.
        
        Returns
        -------
        supports : bool
            Availability.
            
        Notes
        -----
        Other functions such as is_support_[Com] are same format.
        '''
        return True
    def is_support_H(self): return True
    def is_support_M(self): return True
    def is_support_E(self):
        if self.__is_30X() or self.__is_MACX(): return True
        return False
    def is_support_K(self):
        if self.__is_30X() or self.__is_MACX(): return True
        return False
    def is_support_J(self): return True
    def is_support_G(self): return True
    def is_support_R(self): return True
    def is_support_L(self): return True
    def is_support_D(self): return True
    def is_support_V(self):
        if self.__is_30X() or self.__is_MACX(): return False
        return True
    def is_support_U(self):        
        if self.__is_30X(): return True
        return False
    def is_support_W(self):
        if self.__is_30X() or self.__is_MACX(): return True
        return False
    def is_support_T(self):
        if self.__is_30X(): return True
        return False
    def is_support_C(self): return True
    def is_support_S(self):
        if self.__is_MACX(): return False
        return True
    # Status operations
    def is_support_Q(self): return True
    def is_support_Ex(self):return True
    def is_support_Qu(self): return True
    def is_support_QuV(self): return True
    def is_support_QuP(self): return True
    def is_support_QuS(self):
        if self.__is_MACX(): return False
        return True
    def is_support_QuD(self): return True
    def is_support_QuB(self):
        if self.__is_30X() or self.__is_MACX(): return False
        return True
    def is_support_QuM(self):
        if self.__is_30X() or self.__is_MACX(): return True
        return False
    def is_support_QuA(self):
        if self.__is_30X() or self.__is_MACX(): return True
        return False
    def is_support_QuO(self): #Unused
        if self.__is_30X() or self.__is_MACX(): return True
        return False
    def is_support_QuW(self): #Unused
        if self.__is_30X() or self.__is_MACX(): return True
        return False
    def is_support_QuK(self): #Unused
        if self.__is_30X() or self.__is_MACX(): return True
        return False
    def is_support_QuE(self): #Unused
        if self.__is_30X() or self.__is_MACX(): return True
        return False
    def is_support_QuName(self):
        if self.__is_MACX(): return True
        return False
    def is_support_QuVer(self):
        if self.__is_MACX(): return True
        return False    
    # I/O and others
    def is_support_O(self): return True  #Unused
    def is_support_I(self): return True  #Unused
    def is_support_P(self):
        if self.__is_30X(): return True   #Unused
        return False
        

   
class Session:
    '''Session of controlling OptoSigma's stage.
    
    Parameters
    ----------
    ctype : Controllers
        Controller type.
    verbose_level : int, optional
        More messages are output as higher value is set.
    wait_time : int, optional
        Polling time while waiting busy status.
        
    Attributes
    ----------
    controller : Controller
        Parameters of the controller.
    stages : array of Stages
        Parameters of connected stages.    
    
    '''
    def __init__(self, ctype, verbose_level=0, wait_time=2.):
        self.controller = Controller(ctype)
        self.__d = -1 * len(self.controller.delimiter)
        self.stages = {} #{axis: {'name': str, 'stype': stype, 'division': int}}
        self.verbose_level = verbose_level
        self.wait_time = wait_time
        self.port = None
        self.port_open = False
        self.connected = False
        self.divisions_loaded = False
    
    def __len__(self): return len(self.stages)
    
    def __str__(self):
        port_open = 'Open' if self.port_open else 'Closed'
        connected = 'Connected' if self.connected else 'Disconnect'
        return (f"{connected} through {port_open} port RS232 port '{self.port}', with controller '{self.controller.__name__}'")
                   
    def append_stage(self, stype, name='', axis=None):
        ''' Add a new stage parameter.
        
        Setting stage parameters is not mandatory when all operation is done by in_pulse mode.
        
        Parameters
        ----------
        stype : Stages
            The stage type of a new axis stage.
        '''
        if not isinstance(stype, Stages): raise ValueError(f'Axis type must be {Stages}')
        num_axis = self.controller.get_support_axes()
        if axis is None:
            all_axes = [ax+1 for ax in range(num_axis)]
            taken_axes = list(set([stg.get('axis',0) for stg in self.stages.values()])) #TODO: finish this
            taken_axes.sort()
            free_axes = [ax for ax in all_axes if ax not in taken_axes]
            if free_axes:
                axis = int(free_axes[0])
            else:
                raise ValueError(f"No more free axis on this controller: {self.stages}")
        
        if axis > num_axis: raise ValueError(f"Cannot add more than {num_axis} axes.")
        if axis < 1 : raise ValueError(f"Invalid axis number: {axis}.")
        
        if name:
            taken_names = [stg.get('name','') for stg in self.stages.values()]
            if name in taken_names: raise ValueError(f"Name {name} already taken.")
        else:
            name = f"{stype.name}_{axis}"
            
        self.stages[axis] = {'axis': axis, 'name': name, 'stype': stype}

    def remove_stage(self, axis=None):
        
        keys = [k for k in self.stages if (self.stages[k].get('axis',0) == axis or axis == 'all')]
        for k in keys: self.stages.pop(k)

    def list_ports(self):
        """ Lists serial port names
    
            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        """
        ports = []
        try:
            for port_info in serial.tools.list_ports.comports(): ports.append(port_info.__dict__)
        except:    
            if sys.platform.startswith('win'):
                ports = ['COM%s' % (i + 1) for i in range(256)]
            elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
                # this excludes your current terminal "/dev/tty"
                ports = glob.glob('/dev/tty[A-Za-z]*')
            elif sys.platform.startswith('darwin'):
                ports = glob.glob('/dev/tty.*')
            else:
                raise EnvironmentError('Unsupported platform')
            for port in ports:
                try:
                    s = serial.Serial(port)
                    s.close()
                    ports.append({'device': port, 'name': port})
                except (OSError, serial.SerialException):
                    pass

        return ports
        
           
    def connect(self, portname = 'COM3'):
        ''' Connect to the controller via RS232C.
        
        Parameters
        ----------
        portname : str, optional
            Port name of the RS232C where the controller is connected.
        '''
        if self.port:
            if not self.port.is_open: self.port.open() 
        else:
            try:
                self.port = serial.Serial(port    = portname,
                                     baudrate     = self.controller.baudrate,
                                     bytesize     = self.controller.databit,
                                     parity       = self.controller.parity,
                                     stopbits     = self.controller.stopbit,
                                     timeout      = self.controller.read_timeout,
                                     writeTimeout = self.controller.write_timeout,
                                     rtscts       = self.controller.rtscts)
            except serial.SerialException as err:
                self.port_open = False
                self.connected = False
                if re.search("PermissionError",str(err)):
                    self.__print(f"Could not open port '{portname}' because it is already opened by another process.")
                elif re.search("FileNotFoundError",str(err)):
                    self.__print(f"Could not open port '{portname}' because it does not exist on this machine.")
                else:
                    self.__print(err)
                return self.connected
        
        self.port_open = True
        self.connected = self.test_connection()
        
        return self.connected
    
        
    def disconnect(self):
        '''Disconnect the port and returns True if it was a success'''
        if self.port:
            if self.port.is_open: self.port.close()
            self.port_open = self.port.is_open
            return not self.port_open
        else:
            self.port_open = False
            self.connected = False
            return False

    def test_connection(self, raise_err=False):
        '''Test actual connection with the controller, not just with the COM port'''
        connected = False
        check = "Check that the USB cable is plugged in, and that the COM port is not already taken by another process."
        if self.port:
            check = f"Check that the COM port {self.port} is open."
            if self.port.is_open:
                busy = self.is_ready()
                check = "Check that the Controller is powered ON and the front knob turned to 'USB'."
                if busy is not None: connected = True
        if raise_err and not connected:
            raise IOError("No active connection to the controller. " + check)
        return connected
        
    def __print(self, msg, level=1):
        if level <= self.verbose_level: print(msg)
        
    def __send(self, command, no_response=True):
       if not self.port_open:
           raise Bad_Command_Parameter_or_Timing('Not connected to the controller yet.')
       self.port.write(command.encode('ascii') + self.controller.delimiter)
       if self.controller.comm_ack is Comm_ack.MAIN or not no_response:
          response = (self.port.readline()[:self.__d]).decode('utf-8')
          self.__print(command + ' >> ' + response, level=2)
          if response == 'NG':
              raise Bad_Command_Parameter_or_Timing(command + ' failed.')
          return response
       self.__print('[SUB] ' + command, level=2)
    
    
    def __get_axes_by_ref(self, ref, unpack_all=True):
        
        axes = ref
        if ref == 'all':
            axes = [stg.get('axis') for stg in self.stages.values()] if unpack_all else 'all'
        elif isinstance(ref, str):
            axes = [stg.get('axis') for stg in self.stages.values() if stg.get('name') == ref]
            if not axes: raise ValueError(f"Invalid stage reference: {ref}")
            axes = axes[0]
        elif hasattr(ref, '__iter__'):
            axes = []
            for r in ref:
                if isinstance(r, str):
                    ax = [stg.get('axis') for stg in self.stages.values() if stg.get('name') == r]
                    if not ax: raise ValueError(f"Invalid stage reference: {r}")
                    ax = ax[0]
                else:
                    ax = r
                axes.append(ax)
        return axes
                
    
    def is_busy(self, stage=None, func=lambda x:x):
        ''' Whether the controller is under operation or not.
        
        If the controller is busy, only stop and status retrieve commands are accepted and other commands will fail.
        
        Parameters
        ----------        
        stage : int, tuple, list, or None, optional
            If the controller is HIT instruction set, busy status of each slave can be obtainded.
            In this case, specify the slave to get status. If None, the first slave is obtained.
        func : callable
            In HIT mode and multiple stages are specified, busy statuses can be summarized.
            Set all or any for reduce results. Otherwise, each status is returnd by tuple.
        
        Returns
        -------
        busy : bool
            The controller is busy or not.
            
        See also
        --------
        is_ready()
        '''
        axis = self.__get_axes_by_ref(stage)
        if self.controller.is_SHOT() or self.controller.is_MAC():
            return self.__is_busy_shot()
        else:
            axis
            raise NotImplementedError # TODO: HIT mode.
        
    def __is_busy_shot(self):
        if not self.controller.is_support_Ex():
            if not self.controller.is_support_Q():
                raise Not_Supported("Status request Q command not supported.")
            return self.get_status()[3].startswith('B')
        ans = self.__send('!:', no_response=False)
        if ans:
            return ans.startswith('B')
        else:
            return None
        
        
    def is_ready(self, stage=None, func=lambda x:x):
        ''' Whether the controller is ready for operation or not.
        
        If the controller is ready, all commands are acceptable, otherwise limited.
        
        Parameters
        ----------        
        stage : int, tuple, list, or None, optional
            If the controller is HIT instruction set, busy status of each slave can be obtainded.
            In this case, specify the slave to get status. If None, the first slave is obtained.
        func : callable
            In HIT mode and multiple stages are specified, busy statuses can be summarized.
            Set all or any for reduce results. Otherwise, each status is returnd by tuple.
        
        Returns
        -------
        busy : bool
            The controller is ready or not.
            
        See also
        --------
        is_busy()
        '''
        axis = self.__get_axes_by_ref(stage)
        if self.controller.is_SHOT() or self.controller.is_MAC():
            return self.__is_ready_shot()
        else:
            axis
            raise NotImplementedError # TODO: HIT mode.
        
    def __is_ready_shot(self):
        if not self.controller.is_support_Ex():
            if not self.controller.is_support_Q():
                raise Not_Supported("Status request Q command not supported.")
            return self.get_status()[3].startswith('R')
        ans = self.__send('!:', no_response=False)
        if ans:
            return ans.startswith('R')
        else:
            return None
       
    def __wait_for_ready(self, axis=None, func=lambda x:x):
       self.__print('Waiting.')
       while self.is_busy(stage=axis, func=func):
          self.__print('     sleep ' + str(self.wait_time), level=2)
          time.sleep(self.wait_time)
       
    def __load_divisions(self): # TODO: register divisions as a value in the axis dict
        try:
            if self.controller.is_SHOT():
                self.__load_divisions_shot()
            elif self.controller.is_MAC():
                for stg in self.stages: self.stages[stg]['division'] = self.controller.get_default_division()
                self.divisions_loaded = True                
            else:
                raise NotImplementedError #TODO: HIT mode.
        except Exception as err:
            self.__print(f"Could not load divisions from controller because of:{err}", level=2)
            self.__print("Falling back to software-defined values", level=2)
            for stg in self.stages: self.stages[stg]['division'] = self.controller.get_default_division()
            self.divisions_loaded = True
            
    def __load_divisions_shot(self):
        self.__print('Load division settings.', level=2)
        if not self.controller.is_support_Qu(): raise Not_Supported("Querying not supported.")
        if not self.controller.is_support_QuS(): raise Not_Supported("Status query with S is not supported.")
        response = self.__send('?:SW', no_response=False)
        if response:
            divisions = [int(re.sub('\s','',v)) for v in response.split(',')]
            for stg,div in zip(self.stages, divisions): stg['division'] = div
            self.divisions_loaded = True
        else:
            print("Could not get divisions with command '?:SW'")
        
    def __direction(self, pulse):
        return '-' if pulse < 0 else '+'
    
    def get_ID(self):
        controller_ID = []
        if self.controller.is_support_QuName():
            controller_ID += [self.__send('?:S', no_response=False)]
        if self.controller.is_support_QuVer():    
            controller_ID += [self.__send('?:V', no_response=False)]
        return ' '.join(controller_ID)
    
    def reset(self, stage=1, mechanical=False, wait_for_finish=False):
        ''' Reset or initialize the position of stages.
        
        Parameters
        ----------
        stage : int, optional
            Target stage number. if none or 'all', all stages
            If True, all stages are reset simultaneously. In case mechanical is False, all stage parameters must be set.
        mechanical : bool, optional
            If True, stages will reset to the mechanical origin, otherwise, to the electrical zero point.
        wait_for_finish : bool, optional
            If True, check status and wait for operation finish.
            
        See also
        --------
        initialize(), move()
        '''
        if self.controller.is_SHOT():
            axis = self.__get_axes_by_ref(stage, unpack_all=False)
            self.__reset_shot(axis, mechanical, wait_for_finish)
        elif self.controller.is_MAC(): # Using H:W with unconnected stages blocks the MAC controller in the busy state
            axis = self.__get_axes_by_ref(stage, unpack_all=True)
            if not hasattr(axis,'__iter__'): axis = [axis]
            for ax in axis: self.__reset_shot(ax, mechanical, wait_for_finish)
        elif self.controller.is_HIT():
            raise NotImplementedError # TODO: HIT-mode.
            
    def __reset_shot(self, axis, mechanical, wait_for_finish):
        all_axes = True if axis == 'all' else False
        if mechanical:
            if not self.controller.is_support_H():
                raise Not_Supported("Mechanical origin command H not supported.")
            if all_axes:
                self.__print('Mechanical reset, all axes.')
                self.__send('H:W')
            else:
                self.__print('Mechanical reset, #' + str(axis))
                self.__send('H:' + str(axis))
        else:
            if not (self.controller.is_support_A() and self.controller.is_support_G()):
                raise Not_Supported("Driving command A or G not supported.")
            if all_axes:
                self.__print('Electrical reset, ' + str(len(self.stages)) + ' axes')
                com = ['A:W']
                for i in range(len(self.stages)): com.append(['+P0'])
                self.__send(''.join(com))
                self.__send('G:')
            else:
                self.__print('Electrical reset, #' + str(axis))
                self.__send('A:'+str(axis)+'+P0')
                self.__send('G:')
                
        if wait_for_finish:
            self.__wait_for_ready()
        
            
    def initialize(self):
        ''' Initialize all stages at the mechanical origin. '''
        self.reset(stage='all', mechanical=True, wait_for_finish=True)
                
    def move(self, stage=1, amount=0, in_pulse=False, absolute=False, wait_for_finish=True):
        '''Move stages.
        
        Parameters
        ----------
        stage : int
            Target operation stage number. When amount is tuple or list, this value is ignored.
        amount : int or (tuple or list) of int, in motor pulses
            Amount of the transition. When a iterable values are specified, corresponding stages are moved simultaneously.
        in_pulse : bool, optional
            If False, the unit of amount is micro-meters/milli-degrees, otherwise, the unit is pulses.
        absolute : bool, optional
            Represents specified values are absolute position or relative travel.
        wait_for_finish : bool, optional
            If True, check status and wait for operation finish.
            
        See also
        --------
        reset(), jog(), stop(), abort()
        '''
        axis = self.__get_axes_by_ref(stage)
        if self.controller.is_SHOT() or self.controller.is_MAC():
            self.__move_shot(axis, amount, in_pulse, absolute, wait_for_finish)
        else:
            raise NotImplementedError # TODO: HIT-mode.
            
    def __move_shot(self, axis, amount, in_pulse, absolute, wait_for_finish):
        if not self.controller.is_support_G(): raise Not_Supported("Drive command G not supported.")
        msg = ['Move']
        if absolute:
            if not self.controller.is_support_A(): raise Not_Supported("Absolute command A not supported.")
            com = ['A:']
            msg.append('absolutely,')
        else:
            if not self.controller.is_support_M(): raise Not_Supported("Set pulses command M not supported.")
            com = ['M:']
            msg.append('relatively,')
        if isinstance(amount, (list, tuple)):
            com.append('W')
            msg.append(str(len(amount))+' axes,')
            msg.extend([str(m) for m in amount])
            if not in_pulse:
                msg.append('micro-meters/degrees.')
                if not self.divisions_loaded:
                    self.__load_divisions()
                amount = [v * self.stages[ax].division / self.stages[ax]['stype'].baserate for ax, v in zip(axis, amount)]
            com.extend([self.__direction(v)+'P'+str(abs(v)) for v in [int(m) for m in amount]])
            self.__print(' '.join(msg))
            self.__send(''.join(com))
            self.__send('G:')
        else:
            com.append(str(axis))
            com.append(self.__direction(amount))
            com.append('P')
            msg.append('#'+str(axis)+', '+str(amount))
            if not in_pulse:
                msg.append('micro-meters/degrees.')
                if not self.divisions_loaded:
                    self.__load_divisions()
            self.__print('Amount: '+str(amount)+' Base rate: '+str(self.stages[axis]['stype'].baserate) + ' Division:' + str(self.stages[axis]['division']), level=3)
            amount *= self.stages[axis]['division'] / self.stages[axis]['stype'].baserate if not in_pulse else 1
            com.append(str(abs(int(amount))))
            self.__print(' '.join(msg))
            self.__send(''.join(com))
            self.__send('G:')
            
        if wait_for_finish:
            self.__wait_for_ready()
            
    def jog(self, stage=1, directions=1):
        '''Jog drive. Continue moving before arriving the limit point or system limit.
        
        The drive speed is the slowest speed (S-speed).
        To stop the drive, use stop() method.
        
        Jog operation stops automatically when \pm 268,435,455 pulses are sent.
        When the position of a stage becomes \pm 999,999,999, operation is aborted and become a overflow alert mode.
        In this case, call stop() and set_origin() methods to get back to a normal mode.
        
        Parameters
        ----------
        stage : int
            The target stage number. If directions are given by tuple or list, this value is ignored.
        directions : int, tuple, or list
            Jog drive directions. Only a sign of a number is used.
            
        See also
        --------
        stop(), abort(), get_status(), set_origin()
        '''
        axis = self.__get_axes_by_ref(stage)
        if self.controller.is_SHOT() or self.controller.is_MAC():
            self.__jog_shot(axis, directions)
        else:
            raise NotImplementedError # TODO: HIT-mode.
        
    def __jog_shot(self, axis, directions):
        if not self.controller.is_support_J(): raise Not_Supported("Jog command J not supported.")
        if not self.controller.is_support_G(): raise Not_Supported("Drive command G not supported.")
        if isinstance(directions, (tuple, list)):
            arg = [self.__direction(d) for d in directions]
            com = 'J:W' + ''.join(arg)
            self.__print('Jog drive, multi-axes.')
            self.__send(com)
            self.__send('G:')
        else:
            com = 'J:'+str(axis)+self.__direction(directions)
            self.__print('Jog drive, single axis.')
            self.__send(com)
            self.__send('G:')
        
            
    def stop(self, stage=1, emergency=False):
        ''' Stop stages.
        
        Parameters
        ----------
        stage : int
            Target stage number. If emergency is used, this value is ignored.
            If 'all' or None, stop all stages.
        emergency : bool, optional
            Force stop all stages immediately. All operations are aborted. 
            This may cause a big reaction when a stage is moving fast. 
            Also, this mode does not check the capability of the controller to this operation.
            
        See also
        --------
        abort(), jog()
        '''
        axis = self.__get_axes_by_ref(stage, unpack_all=False)
        if self.controller.is_SHOT() or self.controller.is_MAC():
            self.__stop_shot(axis, emergency)
        else:
            raise NotImplementedError # TODO: HIT-mode.
        
    def __stop_shot(self, axis, emergency):
        all_axes = True if (not axis or axis == 'all') else False
        if emergency:
            self.__send('L:E')
            self.__print('Emergency stop.')
            return
        if not self.controller.is_support_L(): raise Not_Supported("Decelerate command L not supported.")
        if all_axes:
            self.__print('Stop All axes.')
            self.__send('L:W')
        else:
            self.__print('Stop axis #'+str(axis))
            self.__send('L:'+str(axis))
        self.__wait_for_ready()
        
        
    def abort(self):
        ''' Equivalent to stop(emergency = True).'''
        self.stop(emergency=True)
            
    def set_origin(self, stage=1):
        ''' Set the electrical zero point of a stage at current position.
        
        Parameters
        ----------
        stage : int
            Target stage number. If None or 'all', applied to all stages
            
        See also
        --------
        reset(), initialize()
        '''
        axis = self.__get_axes_by_ref(stage, unpack_all=False)
        if self.controller.is_SHOT() or self.controller.is_MAC():
            self.__set_origin_shot(axis)
        else:
            raise NotImplementedError # TODO: HIT-mode.
            
    def __set_origin_shot(self, axis):
        if not self.controller.is_support_R(): raise Not_Supported("Set electronic origin command R not supported.")
        all_axes = True if (axis is None or axis == 'all') else False
        if all_axes:
            self.__print('Set origin for all axes.')
            self.__send('R:W')
        else:
            self.__print('Set origin for axis #'+str(axis))
            self.__send('R:'+str(axis))
        

    def __check_SFR(self, s, f, r):
        '''Check the validity of speed settings'''
        s_lim, f_lim, r_lim = self.controller.get_support_speed_ranges()    
        f = max(f_lim[0], min(f_lim[1], f))
        s = max(s_lim[0], min(s_lim[1], s, f))
        r = max(r_lim[0], min(r_lim[1], r))
        if f >= 8000 and s < 64: s = 64
        return s, f, r
            
    def set_speed(self, stage=1, S=1000, F=10000, R=100):
        ''' Set drive speed.
        
        Parameters
        ----------
        stage : int
            Target stage number. If S, F, R are specified by tuple or list, this value is ignored.
        S, F, R : int, tuple, or list
            Speed parameters of each stage.
            S: the slowest speed, F: the fastest speed, R: acceleration and deceleration time.
            
        See also
        --------
        set_speed_reset_drive()
        '''
        axis = self.__get_axes_by_ref(stage)
        if self.controller.is_SHOT() or self.controller.is_MAC():
            self.__set_speed_shot(axis, S, F, R)
        else:
            raise NotImplementedError #TODO: HIT mode.
            
    def __set_speed_shot(self, axis, S, F, R):
        if not self.controller.is_support_D(): raise Not_Supported("Speed setting command D not supported.")
        if isinstance(S, (tuple, list)):
            self.__print('Set speed of multi-axes.')
            SFRs = [self.__check_SFR(s, f, r) for s, f, r in zip(S, F, R)]
            arg = ['S'+str(s)+'F'+str(f)+'R'+str(r) for s, f, r in SFRs]
            com = 'D:W' + ''.join(arg)
            self.__send(com)
        else:
            self.__print('Set speed, #'+str(axis))
            s, f, r = self.__check_SFR(S, F, R)
            com = 'D:'+str(axis)+'S'+str(s)+'F'+str(f)+'R'+str(r)
            self.__send(com)
            
    def set_speed_reset_drive(self, stage=1, S=1000, F=10000, R=100):
        ''' Set drive speed of going back to the origin.
        
        Parameters
        ----------
        stage : int
            Target stage number. If S, F, R are specified by tuple or list, this value is ignored.
        S, F, R : int, tuple, or list
            Speed parameters of each stage.
            Units: pps (pulse per second), pps, ms
            S: the slowest speed, F: the fastest speed, R: acceleration and deceleration time.
            
        See also
        --------
        set_speed()
        '''
        axis = self.__get_axes_by_ref(stage)
        if self.controller.is_SHOT() or self.controller.is_MAC():
            self.__set_speed_reset_drive_shot(axis, S, F, R)
        else:
            raise NotImplementedError # TODO: HIT-mode.
            
    def __set_speed_reset_drive_shot(self, axis, S, F, R):
        if not self.controller.is_support_V(): raise Not_Supported("Set persistent speed command V not supported.")
        if isinstance(S, (tuple, list)):
            self.__print('Set speed of multi-axes.')
            SFRs = [self.__check_SFR(s, f, r) for s, f, r in zip(S, F, R)]
            arg = ['S'+str(s)+'F'+str(f)+'R'+str(r) for s, f, r in SFRs]
            com = 'V:W' + ''.join(arg)
            self.__send(com)
        else:
            self.__print('Set speed of axis #'+str(axis))
            s, f, r = self.__check_SFR(S, F, R)
            com = 'V:'+str(axis)+'S'+str(s)+'F'+str(f)+'R'+str(r)
            self.__send(com)
            
    def set_excitation_mode(self, stage=1, mode=Excitation.Hold):
        '''Set excitation of the moters.
        
        When releasing excitation, stages can be moved manually.
        
        Parameters
        ----------
        mode : Excitation
            Excitation mode.
        '''
        axis = self.__get_axes_by_ref(stage, unpack_all=False)
        if self.controller.is_SHOT() or self.controller.is_MAC():
            self.__set_excitation_mode_shot(axis, mode)
        else:
            raise NotImplementedError # TODO: HIT-mode.
            
    def __set_excitation_mode_shot(self, axis, mode):
        if not self.controller.is_support_C(): raise Not_Supported("Motor excitation command C not supported.")
        all_axes = True if (axis is None or axis == 'all') else False
        if all_axes:
            self.__print('Set all axes to '+mode.name)
            self.__send('C:W'+str(int(mode)))
        else:
            self.__print('Set axis #'+str(axis)+' to '+mode.name)
            self.__send('C:'+str(axis)+str(int(mode)))
        
    def set_division(self, stage=1, division=2):
        ''' Set division of step angle of a stepping motor.
        
        Parameters
        ----------
        stage : int
            Target stage number.
        division : int
            Divisions of a stepping moter.
            When a stage is operated by closed-loop method, Higher value is recommended.
        '''
        axis = self.__get_axes_by_ref(stage)
        if hasattr(axis, '__iter__'): raise ValueError("Cannot set divisions for more than 1 stage at a time")
        if self.controller.is_SHOT():
            self.__set_division_shot(axis, division)
        elif self.controller.is_MAC():
            self.stages[axis]['division'] = division
            self.__print(f"Set stage #{stage}'s division to: {division}")
        else:
            raise NotImplementedError # TODO: HIT-mode.
    
    def __set_division_shot(self, axis, division):
        if not self.controller.is_support_S(): raise Not_Supported("Division query with S not supported.")
        if not division in self.controller.get_support_divisions(): raise ValueError('Unsupported division value.')
        self.__print('Set division of #'+str(axis)+' to '+str(division))
        self.__send('S:' + str(axis) + str(division))
        self.__load_divisions()
        
    def get_status(self):
        ''' Get status of stages and the controller.
        
        Returns
        -------
        positions : list
            List of positions of stages.
        ack1 : str
            'X' or 'K', which represent that the command is denined or accepted, respectively.
        ack2 : str
            'K' represents all stages are stable, and other strings represent one or more stages are stopped at limit sensor.
        ack3 : str
            'B' or 'R', which represent the controller is busy or ready, respectively.
            
        See also
        --------
        get_position(), is_busy(), is_ready()
        '''
        if self.controller.is_SHOT() or self.controller.is_MAC():
            return self.__get_status_shot()
        else:
            raise NotImplementedError # TODO: HIT mode.
        
    def __get_status_shot(self):
        if not self.controller.is_support_Q(): raise Not_Supported("Status command Q not supported")
        response = self.__send('Q:', no_response=False)
        data = response.split(',')
        ack1, ack2, ack3 = data[-3:]
        positions = [int(re.sub('\s','',d)) for d in data[:len(data) - 3]] # Because MAC-04 inserts spaces after the minus sign
        return positions, ack1, ack2, ack3
        
    def get_positions(self, in_pulse=False):
        ''' Get positions of stages.
        
        Parameters
        ----------
        in_pulse : bool, optional
            If True, returns a position as is (pulses), otherwise, returns in micro-meters or milli-degrees.
            
        Returns
        -------
        positions : list
            List of positions of stages.
            
        See also
        --------
        get_status()
        '''
        positions =  self.get_status()[0]
        if in_pulse:
            return positions
        else:
            if not self.divisions_loaded: self.__load_divisions()
            return [pos * stg['stype'].baserate / stg['division'] for pos, stg in zip(positions, self.stages.values())]

#%%
        
def __test_304GS_SGSP46(close_on_quit=True):
    stages = Session(Controllers.SHOT_304GS, verbose_level=3)
    try:
        stages.append_stage(Stages.SGSP46_800)
        stages.connect()    
        if stages.test_connection(raise_err=True):    
            STAGE = 1
            stages.connect()
            stages.initialize()
            stages.set_speed(STAGE, 1, 1000, 10000, 500)
            stages.move(stage=STAGE, amount=100000, wait_for_finish=True)
            stages.move(stage=STAGE, amount=200000, wait_for_finish=True, absolute=True)
            stages.set_origin(stage=STAGE)
            stages.jog(stage=STAGE)
            time.sleep(10)
            stages.stop(stage=STAGE)
            stages.get_positions()
            stages.reset(stage=STAGE, wait_for_finish=True)
            stages.move(stage=STAGE, amount=-200000, wait_for_finish=True, absolute=True)
    except Exception as err:
        stages.disconnect()
        raise err
    if close_on_quit:
        stages.disconnect()
        return
    else:
        return stages
    
def __test_702_SGSP120Y(close_on_quit=True):
    stages = Session(Controllers.SHOT_702, verbose_level=3)
    try:
        stages.append_stage(Stages.SGSP_120YAW)
        stages.connect()
        if stages.test_connection(raise_err=True):
            STAGE = 1
            stages.initialize()
            stages.set_speed(STAGE, 1000, 10000, 500)
            stages.move(stage=STAGE, amount=45000, wait_for_finish=True)
            stages.move(stage=STAGE, amount=90000, wait_for_finish=True, absolute=True)
            stages.set_origin(stage=STAGE)
            stages.jog(stage=STAGE)
            time.sleep(10)
            stages.stopstage(stage=STAGE)
            stages.get_positions()
            stages.reset(stage=STAGE,wait_for_finish=True)
            stages.move(stage=STAGE, amount=-90000, wait_for_finish=True, absolute=True)
            print("TEST PASSED.")
    except Exception as err:
        stages.disconnect()
        raise err
    if close_on_quit:
        stages.disconnect()
        return
    else:
        return stages
    
def __test_MAC04_OSMS120YAW(close_on_quit=True):
    stages = Session(Controllers.MAC_04, verbose_level=1)
    try:
        stages.append_stage(Stages.OSMS_120YAW)
        stages.connect('COM3')
        if stages.test_connection(raise_err=True):
            print(stages.get_ID())
            stages.set_division(stage=1, division=2)
            STAGE = 1
            stages.initialize()
            stages.set_speed(stage=STAGE, S=500, F=4000, R=500) # in pulses per second, and ms
            stages.move(stage=STAGE, amount=500, wait_for_finish=True)
            stages.move(stage=STAGE, amount=20000, wait_for_finish=True, absolute=True)
            stages.set_origin(stage=STAGE)
            stages.jog(stage=STAGE)
            time.sleep(5)
            stages.stop(stage=STAGE)
            print(f"Positions: {stages.get_positions()}")
            stages.reset(stage=STAGE, wait_for_finish=True)
            stages.move(amount=-20000, wait_for_finish=True, absolute=True)
            print("TEST PASSED.")
    except Exception as err:
        stages.disconnect()
        raise err
    if close_on_quit:
        stages.disconnect()
        return
    else:
        return stages

def __test_MAC04_OSMSMDHM200(close_on_quit=True):
    stages = Session(Controllers.MAC_04, verbose_level=1)
    try:
        stages.append_stage(Stages.MHDM_200, axis=1, name='Rotation')
        stages.append_stage(Stages.MHDM_200)
        stages.connect('COM3')
        if stages.test_connection(raise_err=True):
            print(stages.get_ID())
            stages.set_division(stage=1, division=2)
            STAGE = 1
            stages.initialize()
            stages.set_speed(stage=STAGE, S=500, F=4000, R=500) # in pulses per second, and ms
            stages.move(stage=STAGE, amount=500, wait_for_finish=True)
            stages.move(stage=STAGE, amount=5000, wait_for_finish=True, absolute=True)
            stages.set_origin(stage=STAGE)
            stages.jog(stage=STAGE)
            time.sleep(5)
            stages.stop(stage=STAGE)
            print(f"Positions: {stages.get_positions()}")
            stages.reset(stage=STAGE, wait_for_finish=True)
            stages.move(amount=-5000, wait_for_finish=True, absolute=True)
            print("TEST PASSED.")
    except Exception as err:
        stages.disconnect()
        raise err
    if close_on_quit:
        stages.disconnect()
        return
    else:
        return stages
    
if __name__ == '__main__':
#    __test_304GS_SGSP46()
#    __test_702_SGSP120Y()
    stages = __test_MAC04_OSMS120YAW(close_on_quit=False)
    pass
