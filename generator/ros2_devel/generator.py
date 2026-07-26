# MIT License
#
# Copyright (c) [2019] [Angelo Ferrando]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__),'code/monitor/'))
import yaml
import xml.etree.ElementTree as ET
from jedi.inference.names import AbstractNameDefinition
from prompt_toolkit.layout.controls import GetLinePrefixCallable
from gi._gtktemplate import Child
import setup_resources as sr


class CodeGenAndROSUtils():

    # just for code gent 
    def __init__(self):
        self.indent_level = 0 
        self.new_line = '\n'
        self.debug = False
        
    def reset_indent(self,msg):
        if self.debug:
            print(msg+" Resetting indent level from {0} to 0".format(self.indent_level))
        self.indent_level=0
        
    def check_indent(self,msg):
        if self.debug:
            print(msg+" Indent level: {0}".format(self.indent_level))


    def inc_indent(self, current_indent):
        self.indent_level+=1
        return current_indent + "\t"
    
    def dec_indent(self, current_indent):
        if self.indent_level <=0:
            print("Error with the indentation perhaps, trying to decrement an indent incorrectly")
        if current_indent == '':
            return current_indent
        else:
            # from https://stackoverflow.com/questions/2556108/rreplace-how-to-replace-the-last-occurrence-of-an-expression-in-a-string
            toremove = "\t"
            replacewith = ""
            maxreplace = 1
            new_indent = replacewith.join(current_indent.rsplit(toremove, maxreplace))
            self.indent_level-=1
            return new_indent
    
    
    ''' this is an array of lines'''

    def write_lines(self, lines, monitor_id,monloc):
        with open( monloc+monitor_id + '.py', 'w') as mon:
            for l in lines:
                mon.write(l)
        
    def create_python_header(self):
        return '#!/usr/bin/env python\n'    
     
     
    def append_lines_to_list_with_prefix(self,linelist,lines,lineprefix):
        for l in lines:
            linelist.append(lineprefix+l)
        return linelist
    
        
    # ROS things 
    def get_ros_info_logging_line(self, text):
        return 'self.get_logger().info({0})\n'.format(text)
    
    def get_ros_time_line(self):
        return 'float(self.get_clock().now().to_msg().sec)'

        
        
        
    def ros_subscriber_creation_command(self,subname,subtype,callbackname,qsize,doStringName=True):
        if doStringName:
            return "self.create_subscription(topic='{tname}',msg_type={ttype},callback={cbname},qos_profile={qs})\n".format(tname=subname,ttype=subtype,cbname=callbackname,qs=qsize)
        else:
            return "self.create_subscription(topic={tname},msg_type={ttype},callback={cbname},qos_profile={qs})\n".format(tname=subname,ttype=subtype,cbn=callbackname,qs=qsize)    
        
    def ros_server_service_creation_command(self,srvname,srvtype,callbackname,doStringName=True):
        if doStringName:
            return "self.create_service({srvtype}, '{srvname}', {cbname}, callback_group=MutuallyExclusiveCallbackGroup())\n".format(srvname=srvname,srvtype=srvtype,cbname=callbackname)
        else:
            return "self.create_service({srvtype}, {srvname}, {cbname},callback_group=MutuallyExclusiveCallbackGroup())\n".format(srvname=srvname,srvtype=srvtype,cbname=callbackname)
        
        
    def ros_publisher_creation_command(self, pubname, pubtype, qsize, doStringName=True):
        if doStringName:
            return "self.create_publisher(topic='{tname}',msg_type={ttype},qos_profile={qs})\n".format(tname=pubname, ttype=pubtype, qs=qsize)
        else:
            return "self.create_publisher(topic={tname},msg_type={ttype},qos_profile={qs})\n".format(tname=pubname, ttype=pubtype, qs=qsize)
        
    def ros_client_service_creation_command(self, srvname, srvtype, doStringName=True):
        if doStringName:
            return "self.create_client({srvtype}, '{srvname}', callback_group=MutuallyExclusiveCallbackGroup())\n".format(srvname=srvname, srvtype=srvtype)
        else:
            return "self.create_client({srvtype}, {srvname}, callback_group=MutuallyExclusiveCallbackGroup())\n".format(srvname=srvname, srvtype=srvtype)
    
            
        
class MonitorGenerator():
    
    
    # initialising variable names for the class mostly 
    def __init__(self):
        
        self.queue_size = 1000
        self.mon_pubs_dict_name = 'self.monitor_publishers'
        self.config_pubs_dict_name = 'self.config_publishers'
        self.config_subs_dict_name = 'self.config_subscribers'
        self.config_client_srvs_dict_name = 'self.config_client_services'
        self.config_server_srvs_dict_name = 'self.config_server_services'
        self.messages_dict_name = 'self.dict_msgs'
        self.threading_loc_name = 'self.ws_lock'
        self.websocket_name = 'self.ws'
        self.logging_fname = 'self.logging'
        self.message_received_fname_topic = 'self.on_message_topic'
        self.message_received_fname_service_request = 'self.on_message_service_request'
        self.message_received_fname_service_response = 'self.on_message_service_response'
        self.monitor_id_vname = 'self.name'
        self.mon_name_input = 'monitor_name'
        self.log_name_input = 'log'
        self.actions_name_input = 'actions'
        self.actions_vname = 'self.actions'
        self.log_name = 'self.logfn'
        self.pub_topics_name = 'self.publish_topics'
        self.publish_topics = None
        self.topics_info = 'self.topics_info'
        self.services_info = 'self.services_info'
        self.action_goals_info = 'self.action_goals'
        self.codegenutils = CodeGenAndROSUtils()

    # other helpful class related things 
    def get_mon_class_name(self,monitor_id):
        return "ROSMonitor_{0}".format(monitor_id)
    
    def create_class_header(self,monitor_id):
        return "class {cn}(Node):\n".format(cn = self.get_mon_class_name(monitor_id))    

    # things related to publishing and subscribing no function gen here 
    
    ''' get the message types for the topics '''
    def get_topic_and_service_msg_types(self, topics_with_types_and_action):
        tp_lists = {}
        for topic_msg_details in topics_with_types_and_action:
            package = topic_msg_details['type'][0:topic_msg_details['type'].rfind('.')]
            type = topic_msg_details['type'][topic_msg_details['type'].rfind('.') + 1:]
            topic = topic_msg_details['name']
            tp_lists[topic] = {'package':package, 'type':type}
        return tp_lists
     
    def get_subscribers(self, topics_with_types_and_action):
        subscribers = {}
        for tp_info in topics_with_types_and_action:
            if 'publishers' in tp_info:
                subscribers[tp_info['name']] = {'remapped':True, 'callback':True, 'republish':True}
            elif 'subscribers' in tp_info:
                subscribers[tp_info['name']] = {'remapped':False, 'callback':True, 'republish':True}
            else:
                subscribers[tp_info['name']] = {'remapped':False, 'callback':True, 'republish':False}
        return subscribers

    def get_services(self, services_with_types_and_action):
        services = {}
        for srv_info in services_with_types_and_action:
            services[srv_info['name']] = {'remapped':True, 'callback':True, 'republish':True}
        return services

    def get_remapped_name(self, name):
        return name + "_mon"
    #msg_or_srvtype dict{package, type}
    #modifica il nome delle .action in modo usato da ROS2 (classe interna impl)
    def get_runtime_type_expr(self, msg_or_srv_type):
        package = msg_or_srv_type['package']
        type_name = msg_or_srv_type['type']
        if not package.endswith('.action'):
            return type_name
        if type_name.endswith('_FeedbackMessage'):
            action_name = type_name[:-len('_FeedbackMessage')]
            return action_name + ".Impl.FeedbackMessage"
        if type_name.endswith('_SendGoal'):
            action_name = type_name[:-len('_SendGoal')]
            return action_name + ".Impl.SendGoalService"
        if type_name.endswith('_GetResult'):
            action_name = type_name[:-len('_GetResult')]
            return action_name + ".Impl.GetResultService"
        return type_name
    # send goal, cancel, get result sono services, non serve modificare il qos.
    # feedback è gia compatibile con il qos compatibile.
    # Bisogna modificare il qos di status in quanto ROS 2 ne usa uno specifico. (https://design.ros2.org/articles/actions.html#:~:text=occur,The,-possible)
    def get_topic_qos_expr(self, topic_name):
        if topic_name.endswith('/_action/status'):
            return 'qos_profile_action_status_default'
        return str(self.queue_size)

    # functions that generate lines of code but not whole functions
    
    def create_subscriber_line(self,name,tinfo,tmsg_type,cbname):
        tpname = name
        subtype = self.get_runtime_type_expr(tmsg_type)
        qos_expr = self.get_topic_qos_expr(name)
        if tinfo['remapped']:
            tpname = self.get_remapped_name(name)
        line = self.codegenutils.ros_subscriber_creation_command(tpname, subtype, cbname, qos_expr)
        return line
    
    def create_server_service_line(self,name,sinfo,smsg_type,cbname):
        srvname = name
        srvtype = self.get_runtime_type_expr(smsg_type)
        if sinfo['remapped']:
            srvname = self.get_remapped_name(name)
        line = self.codegenutils.ros_server_service_creation_command(srvname, srvtype, cbname)
        return line
    
    # Old one
    # def create_publisher_line(self, name, tinfo, tmsg_type):
    #     if not tinfo['republish']:
    #         return None
    #     tpname = name 
    #     if tinfo['remapped']:
    #         # tpname = self.get_remapped_name(tpname)
    #         pubtype = tmsg_type['type']
    #         line = self.codegenutils.ros_publisher_creation_command(tpname, pubtype, self.queue_size)
    #         return line
    #     else:
    #         return None
    def create_publisher_line(self, name, tinfo, tmsg_type):
        if not tinfo['republish']:
            return None
        tpname = name 
        if not tinfo['remapped']:
            tpname = self.get_remapped_name(tpname)
        pubtype = self.get_runtime_type_expr(tmsg_type)
        qos_expr = self.get_topic_qos_expr(name)
        line = self.codegenutils.ros_publisher_creation_command(tpname, pubtype, qos_expr)
        return line
        
    def create_client_service_line(self, name, srvinfo, srvmsg_type):
        if not srvinfo['republish']:
            return None
        srvname = name 
        if not srvinfo['remapped']:
            srvname = self.get_remapped_name(srvname)
        srvtype = self.get_runtime_type_expr(srvmsg_type)
        line = self.codegenutils.ros_client_service_creation_command(srvname, srvtype)
        return line
    
    def create_config_subscriber_lines(self,subscribers,tp_lists,cbdict):
        lines = []
        for t in subscribers:
            subline = self.create_subscriber_line(t, subscribers[t], tp_lists[t], 'self.'+cbdict[t]['name'])
            line = "{config_sub_name}['{tname}']={subline}\n".format(config_sub_name=self.config_subs_dict_name, tname = t, subline=subline)
            lines.append(line)
            
        return lines
    
    def create_config_server_service_lines(self,services,srv_lists,cbdict):
        lines = []
        for s in services:
            srvline = self.create_server_service_line(s, services[s], srv_lists[s], 'self.'+cbdict[s]['name'])
            line = "{config_srv_name}['{sname}']={srvline}\n".format(config_srv_name=self.config_server_srvs_dict_name, sname = s, srvline=srvline)
            lines.append(line)
            
        return lines
            
            
    def create_config_publishers(self,subscribers,tp_lists):
        publishers={}
        for topic in subscribers:
            publishers[topic] = self.create_publisher_line(topic, subscribers[topic], tp_lists[topic])
            
        return publishers
    
    def create_config_publishers_lines(self,subscribers,tp_lists):
        lines = []
        for t in subscribers:
            publine = self.create_publisher_line(t, subscribers[t], tp_lists[t])
            if publine is not None:
                line = "{config_pub_dname}['{tname}']={publine}\n".format(config_pub_dname=self.config_pubs_dict_name,tname=t,publine=publine)
                lines.append(line)
        if len(lines) == 0:
            self.publish_topics = False
        else:
            self.publish_topics = True
        return lines

    def create_config_client_services_lines(self,services,srv_lists):
        lines = []
        for s in services:
            srvline = "ServiceNode({srvtype},'{srvname}')".format(srvtype=self.get_runtime_type_expr(srv_lists[s]), srvname=s)
            if srvline is not None:
                line = "{config_srvs_dname}['{srvname}']={srvline}\n".format(config_srvs_dname=self.config_client_srvs_dict_name,srvname=s,srvline=srvline)
                lines.append(line)
                # line = "while not {config_srvs_dname}['{srvname}'].wait_for_service(timeout_sec=1.0): self.get_logger().info('service not available, waiting again...')\n".format(config_srvs_dname=self.config_client_srvs_dict_name,srvname=s)
                # lines.append(line)
        return lines
    
    def create_config_services_lines(self,subscribers,tp_lists):
        lines = []
        for t in subscribers:
            publine = self.create_publisher_line(t, subscribers[t], tp_lists[t])
            if publine is not None:
                line = "{config_pub_dname}['{tname}']={publine}\n".format(config_pub_dname=self.config_pubs_dict_name,tname=t,publine=publine)
                lines.append(line)
        if len(lines) == 0:
            self.publish_topics = False
        else:
            self.publish_topics = True
        return lines

        
    def create_logging_func(self):
        self.codegenutils.reset_indent("logging func start")
        lineprefix = ''
        input_var = 'json_dict'
        header = "def {logfuncname}(self,{invar}):\n".format(logfuncname=self.logging_fname.replace("self.", ""),invar=input_var)
        lines=[header]
        
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "try:\n"
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "with open({logname},'a+') as log_file:\n".format(logname=self.log_name)
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "log_file.write(json.dumps({jd})+'\\n')\n".format(jd=input_var)
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        msg = "'Event logged'"
        line = self.codegenutils.get_ros_info_logging_line(msg)
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        
        
        line="except:\n"
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        msg = "'Unable to log the event'"
        line = self.codegenutils.get_ros_info_logging_line(msg)
        lines.append(lineprefix+line)
        
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        
        self.codegenutils.check_indent("logging func done")
        return lines
        

    def create_on_message_topic(self,silent,oracle_action,tp_lists):
        self.codegenutils.reset_indent("on message func start")
        lineprefix =''
        msg_input_var = 'message'
        header ="def {onmsgfunc}(self,{msg_input}):\n".format(onmsgfunc = self.message_received_fname_topic.replace("self.",""),msg_input = msg_input_var)
        lines=[header]
        
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        jsondict = 'json_dict'
        line = "{jd} = json.loads({invar})\n".format(jd=jsondict,invar=msg_input_var)
        lines.append(lineprefix+line)
        line = "verdict = str({jd}['verdict'])\n".format(jd=jsondict)
        lines.append(lineprefix+line)
        line = "if verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':\n"
        lines.append(lineprefix+line)
        
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        
        line = "if verdict == 'true' and not {pt_var}:\n".format(pt_var=self.pub_topics_name)
        lines.append(lineprefix+line)
        lineprefix=self.codegenutils.inc_indent(lineprefix)
        msg = "'The monitor concluded the satisfaction of the property under analysis and can be safely removed.'"
        line = self.codegenutils.get_ros_info_logging_line(msg)
        lines.append(lineprefix+line)
        line = "{ws}.close()\n".format(ws=self.websocket_name)
        lines.append(lineprefix+line)
        line = "exit(0)\n"
        lines.append(lineprefix+line)
        
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        line = "else:\n"
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "{logging_fname}({data_dname})\n".format(logging_fname=self.logging_fname, data_dname=jsondict)
        lines.append(lineprefix+line)
        line = "topic = {jsondict}['topic']\n".format(jsondict=jsondict)
        lines.append(lineprefix+line)
        
        if not silent:
            msg = "'The event '+{data}+' is consistent and republished'".format(data = msg_input_var)
            line = self.codegenutils.get_ros_info_logging_line(msg)
            lines.append(lineprefix+line)
        if oracle_action == 'nothing':
            line = "if topic in {pubdict}:\n".format(pubdict=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix = self.codegenutils.inc_indent(lineprefix)
            line = "{pubdict}[topic].publish({msgdict}[{jsond}['time']])\n".format(pubdict=self.config_pubs_dict_name,msgdict=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
            lineprefix = self.codegenutils.dec_indent(lineprefix)
            line = "del {msgdict}[{jsond}['time']]\n".format(msgdict=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
            lineprefix = self.codegenutils.dec_indent(lineprefix)
        else:
            # lineprefix = self.codegenutils.inc_indent(lineprefix)
            line = "del {jsond}['topic']\n".format(jsond=jsondict)
            lines.append(lineprefix+line)
            line = "del {jsond}['time']\n".format(jsond=jsondict)
            lines.append(lineprefix+line)
            line = "if 'verdict' in {jsond}: del {jsond}['verdict']\n".format(jsond=jsondict)
            lines.append(lineprefix+line)
            line = "ROS_message = eval(''+{topicsinfo}[topic]['type']+'()')\n".format(topicsinfo=self.topics_info)
            lines.append(lineprefix+line)
            line = "rosidl_runtime_py.set_message_fields(ROS_message,{jsond})\n".format(jsond=jsondict)
            lines.append(lineprefix+line)
           
            line = "if topic in {pubdict}:\n".format(pubdict=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix = self.codegenutils.inc_indent(lineprefix)
            line = "{pubdict}[topic].publish(ROS_message)\n".format(pubdict=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix=self.codegenutils.dec_indent(lineprefix)
            lineprefix=self.codegenutils.dec_indent(lineprefix)
        
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        
        # in the else for the first if 
        line = "else:\n"
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        
        # if the verdict is not ture or unknown 
        line = "{logfunc}({jsond})\n".format(logfunc=self.logging_fname,jsond=jsondict)
        lines.append(lineprefix+line)
        
        if not silent:
            msg = "'The event' + {msg} + ' is inconsistent' ".format(msg=msg_input_var)
            line = self.codegenutils.get_ros_info_logging_line(msg)
            lines.append(lineprefix+line)
        
        manylines = ["error = MonitorError()\n",
                     "error.m_topic = {0}['topic']\n".format(jsondict),
                     "error.m_time = {0}['time']\n".format(jsondict),
                     "error.m_property = {0}.get('spec', '')\n".format(jsondict),
                     ]
        lines=self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        
        if oracle_action == 'nothing':
            line = "error.m_content = str({dmsgs}[{jsond}['time']])\n".format(dmsgs=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
        else:
            manylines = ["{jsond}_copy = {jsond}.copy()\n".format(jsond=jsondict),
                         "del {jsond}_copy['topic']\n".format(jsond=jsondict),
                         "del {jsond}_copy['time']\n".format(jsond=jsondict),
                         "if 'spec' in {jsond}_copy: del {jsond}_copy['spec']\n".format(jsond=jsondict),
                         
                         # "del {jsond}_copy['error']\n".format(jsond=jsondict),
                         "error.m_content = json.dumps({jsond}_copy)\n".format(jsond=jsondict)
                         ]
            lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        line = "{monpubs}['error'].publish(error)\n".format(monpubs=self.mon_pubs_dict_name)
        lines.append(lineprefix+line)
        #Quando arriva un feedback o status -> possono essere ricevuti più volte durante l'esecuzione di una action.
        manylines = [
            "if {jsond}.get('event_kind') == 'status' or {jsond}.get('event_kind') == 'feedback':\n".format(jsond=jsondict),
        ]
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix) # aggiungo if sopra alla lista delle righe con indentazione
        lineprefix = self.codegenutils.inc_indent(lineprefix) #incremento indentazione (ci troviamo nella condizione if)
        manylines = self.create_cancel_action_topic_goal_lines(jsondict) # logica di cancel lato status e feedback
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix) #aggoungiamo la riga sopra di logica al monitor
        lineprefix = self.codegenutils.dec_indent(lineprefix) # decremento indentazione (esco dall'if)
        line = "if verdict == 'false' and not {pt_var}:\n".format(pt_var=self.pub_topics_name)
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        
        msg = "'The monitor concluded the violation of the property under analysis and can be safely removed.'"
        line = self.codegenutils.get_ros_info_logging_line(msg)
        lines.append(lineprefix+line)
        line = "{ws}.close()\n".format(ws=self.websocket_name)
        lines.append(lineprefix+line)
        line = "exit(0)\n"
        lines.append(lineprefix+line)
        
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        line = "if {actions}[{jsond}['topic']][0] != 'filter':\n".format(actions=self.actions_vname,jsond=jsondict)
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "topic = {jsond}['topic']\n".format(jsond=jsondict)
        lines.append(lineprefix+line)
        
        if oracle_action == 'nothing':
            line = "if topic in {pubd}:\n".format(pubd=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix = self.codegenutils.inc_indent(lineprefix)
            line = "{pubd}[topic].publish({dmsgs}[{jsond}['time']])\n".format(pubd=self.config_pubs_dict_name, dmsgs=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
            lineprefix = self.codegenutils.dec_indent(lineprefix)
            line = "del {dmsgs}[{jsond}['time']]\n".format( dmsgs=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
            
            
        else:
            manylines = [   "del {jsond}['topic']\n".format(jsond=jsondict),
                         "del {jsond}['time']\n".format(jsond=jsondict),
                         "if 'spec' in {jsond}: del {jsond}['spec']\n".format(jsond=jsondict), #ripubblico solo i dati richiesti da ROS2, elimino i dati aggiunti da ROSMonitor
                         "if 'verdict' in {jsond}: del {jsond}['verdict']\n".format(jsond=jsondict)
                         # "del {jsond}['error']\n".format(jsond=jsondict)
                ]
            lines=self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
            
            line = "ROS_message = eval(''+{topicsinfo}[topic]['type']+'()')\n".format(topicsinfo=self.topics_info)
            lines.append(lineprefix+line)
            line = "rosidl_runtime_py.set_message_fields(ROS_message,{jsond})\n".format(jsond=jsondict)
            lines.append(lineprefix+line)
           
            line = "if topic in {pubdict}:\n".format(pubdict=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix = self.codegenutils.inc_indent(lineprefix)
            line = "{pubdict}[topic].publish(ROS_message)\n".format(pubdict=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix = self.codegenutils.dec_indent(lineprefix)
            
        lineprefix = self.codegenutils.dec_indent(lineprefix)
            
        line="error=True\n"
        lines.append(lineprefix+line)   
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        line = "verdict_msg = String()\n"
        lines.append(lineprefix+line)
        line = "verdict_msg.data = verdict\n"
        lines.append(lineprefix+line)
        line = "{monpubs}['verdict'].publish(verdict_msg)\n".format(monpubs=self.mon_pubs_dict_name)
        lines.append(lineprefix+line)
        
        self.codegenutils.check_indent("on message func done")    
        return lines
    
    def create_on_message_service_request(self,silent,oracle_action,srv_lists,do_oracle):
        self.codegenutils.reset_indent("on message func start")
        lineprefix =''
        msg_input_var = 'message'
        header ="def {onmsgfunc}(self,{msg_input}):\n".format(onmsgfunc = self.message_received_fname_service_request.replace("self.",""),msg_input = msg_input_var)
        lines=[header]
        
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        # line = "if {msg_input} is not None:\n".format(msg_input=msg_input_var)
        # lines.append(lineprefix+line)
        # lineprefix = self.codegenutils.inc_indent(lineprefix)
        jsondict = 'json_dict'
        line = "{jd} = json.loads({invar})\n".format(jd=jsondict,invar=msg_input_var)
        lines.append(lineprefix+line)
        line = "verdict = str({jd}['verdict'])\n".format(jd=jsondict)
        lines.append(lineprefix+line)
        line = "service = {jsondict}['service'] = {jsondict}['service'].replace('_mon', '')\n".format(jsondict=jsondict)
        lines.append(lineprefix+line)
        line = "verdict_msg = String()\n"
        lines.append(lineprefix+line)
        line = "verdict_msg.data = verdict\n"
        lines.append(lineprefix+line)
        line = "{monpubs}['verdict'].publish(verdict_msg)\n".format(monpubs=self.mon_pubs_dict_name)
        lines.append(lineprefix+line)
        line = "if verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':\n"
        lines.append(lineprefix+line)
        
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        # if not do_oracle:
        line = "del {data_dname}['verdict']\n".format(data_dname=jsondict)
        lines.append(lineprefix + line)

        line = "{logging_fname}({data_dname})\n".format(logging_fname=self.logging_fname, data_dname=jsondict)
        lines.append(lineprefix+line)
        manylines = self.create_track_cancel_request_lines(jsondict, "service") #aggiungo logica di cancellazione
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix) #scrivo logica di cancellazione sul monitor
        
        if not silent:
            msg = "'The request '+{data}+' is consistent, the service is called'".format(data = msg_input_var)
            line = self.codegenutils.get_ros_info_logging_line(msg)
            lines.append(lineprefix+line)
    
        line = "if service in {srvdict}:\n".format(srvdict=self.config_client_srvs_dict_name)
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "res = {srvdict}[service].call_service({msgdict}[{jsond}['time']])\n".format(srvdict=self.config_client_srvs_dict_name,msgdict=self.messages_dict_name,jsond=jsondict)
        lines.append(lineprefix + line)
        # line = "rclpy.spin_until_future_complete(self, res)\n"
        # lines.append(lineprefix + line)
        # line = "res = res.result()\n"
        # lines.append(lineprefix + line)
        line = "{jsond}['response'] = rosidl_runtime_py.message_to_ordereddict(res)\n".format(jsond=jsondict)
        lines.append(lineprefix + line)
        manylines = self.create_action_metadata_lines(jsondict, "service", 'response') # aggiuge dati relativi a una action ogni volta che riceve un messaggio relativo ad essa. quale action, fase action, quale goal.
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        line = "{msgdict}[{jsond}['time']] = {input_varname}\n".format(msgdict=self.messages_dict_name, jsond=jsondict, input_varname='res')
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        # We do not consider the Oracle's modification in case of services (maybe in the future)
        # else:
        #     # lineprefix = self.codegenutils.inc_indent(lineprefix)
        #     line = "del {jsond}['service']\n".format(jsond=jsondict)
        #     lines.append(lineprefix+line)
        #     line = "del {jsond}['time']\n".format(jsond=jsondict)
        #     lines.append(lineprefix+line)
        #     line = "if 'verdict' in {jsond}: del {jsond}['verdict']\n".format(jsond=jsondict)
        #     lines.append(lineprefix+line)
        #     line = "ROS_message = eval(''+{servicesinfo}[service]['type']+'()')\n".format(servicesinfo=self.services_info)
        #     lines.append(lineprefix+line)
        #     line = "rosidl_runtime_py.set_message_fields(ROS_message,{jsond}['request'])\n".format(jsond=jsondict)
        #     lines.append(lineprefix+line)
        #     line = "if service in {srvdict}:\n".format(srvdict=self.config_srv_dict_name)
        #     lines.append(lineprefix+line)
        #     lineprefix = self.codegenutils.inc_indent(lineprefix)
        #     line = "res = {srvdict}[service].call({msgdict}[{jsond}['time']])\n".format(srvdict=self.config_client_srvs_dict_name,msgdict=self.messages_dict_name,jsond=jsondict)
        #     lines.append(lineprefix+line)
        #     lineprefix=self.codegenutils.dec_indent(lineprefix)
        #     lineprefix=self.codegenutils.dec_indent(lineprefix)

        oracle_response_varname = 'msg'
        # line = "if 'verdict' in {jsond}: del {jsond}['verdict']\n".format(jsond=jsondict)
        # lines.append(lineprefix+line)
        line = "del {jsond}['request']\n".format(jsond=jsondict)
        lines.append(lineprefix+line)
        # line = "{jsond}['response']=True\n".format(jsond=jsondict)
        # lines.append(lineprefix+line)
        if do_oracle:
            line = "{ws_lock}.acquire()\n".format(ws_lock=self.threading_loc_name)
            lines.append(lineprefix + line)
            line = "{ws}.send(json.dumps({data_dname}))\n".format(ws=self.websocket_name, data_dname=jsondict)
            lines.append(lineprefix + line)
            line = "{msg}={ws}.recv()\n".format(msg=oracle_response_varname, ws=self.websocket_name)
            lines.append(lineprefix + line)
            line = "{ws_lock}.release()\n".format(ws_lock=self.threading_loc_name)
            lines.append(lineprefix + line)
        else:
            line = "{data_dname}['verdict']='currently_true'\n".format(data_dname=jsondict)
            lines.append(lineprefix + line)
            line = "{msg}=json.dumps({data_dname})\n".format(msg=oracle_response_varname, data_dname=jsondict)
            lines.append(lineprefix + line)
        line = "return {msg_fname}({msg_vname})\n".format(msg_fname=self.message_received_fname_service_response, msg_vname=oracle_response_varname)
        lines.append(lineprefix + line)
        
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        
        # in the else for the first if 
        line = "else:\n"
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        
        # if the verdict is not true or unknown 
        line = "{logfunc}({jsond})\n".format(logfunc=self.logging_fname,jsond=jsondict)
        lines.append(lineprefix+line)
        
        if not silent:
            msg = "'The event request' + {msg} + ' is inconsistent' ".format(msg=msg_input_var)
            line = self.codegenutils.get_ros_info_logging_line(msg)
            lines.append(lineprefix+line)
        
        manylines = ["error = MonitorError()\n",
                     "error.m_service = {0}['service'].replace('_mon', '')\n".format(jsondict),
                     "error.m_time = {0}['time']\n".format(jsondict),
                     "error.m_property = {0}.get('spec', '')\n".format(jsondict), # la action prevede più campi della service
                     ]
        lines=self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        
        if oracle_action == 'nothing':
            line = "error.m_content = str({dmsgs}[{jsond}['time']])\n".format(dmsgs=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
        # else:
        #     manylines = ["{jsond}_copy = {jsond}.copy()\n".format(jsond=jsondict),
        #                  "del {jsond}_copy['service']\n".format(jsond=jsondict),
        #                  "del {jsond}_copy['time']\n".format(jsond=jsondict),
        #                  "del {jsond}_copy['spec']\n".format(jsond=jsondict),
                         
        #                  # "del {jsond}_copy['error']\n".format(jsond=jsondict),
        #                  "error.m_content = json.dumps({jsond}_copy)\n".format(jsond=jsondict)
        #                  ]
        #     lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        line = "{monpubs}['error'].publish(error)\n".format(monpubs=self.mon_pubs_dict_name)
        lines.append(lineprefix+line)

        line="error=True\n"
        lines.append(lineprefix+line)   
        lineprefix = self.codegenutils.dec_indent(lineprefix)

        line = "if {actions}[{jsond}['service']][0] != 'filter':\n".format(actions=self.actions_vname,jsond=jsondict)
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "service = {jsond}['service'] = {jsond}['service'].replace('_mon', '')\n".format(jsond=jsondict)
        lines.append(lineprefix+line)
    
        line = "if service in {srvdict}:\n".format(srvdict=self.config_client_srvs_dict_name)
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "res = {srvdict}[service].call_service({msgdict}[{jsond}['time']])\n".format(srvdict=self.config_client_srvs_dict_name,msgdict=self.messages_dict_name,jsond=jsondict)
        lines.append(lineprefix + line)
        # line = "rclpy.spin_until_future_complete(self, res)\n"
        # lines.append(lineprefix + line)
        # line = "res = res.result()\n"
        # lines.append(lineprefix + line)
        line = "{msgdict}[{jsond}['time']] = {input_varname}\n".format(msgdict=self.messages_dict_name, jsond=jsondict, input_varname='res')
        lines.append(lineprefix+line)
        line = "{jsond}['response'] = rosidl_runtime_py.message_to_ordereddict(res)\n".format(jsond=jsondict)
        lines.append(lineprefix + line)
        manylines = self.create_action_metadata_lines(jsondict, "service", 'response') # aggiugne dati alla service response
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
    
        oracle_response_varname = 'msg'
        line = "if 'verdict' in {jsond}: del {jsond}['verdict']\n".format(jsond=jsondict)
        lines.append(lineprefix+line)
        line = "del {jsond}['request']\n".format(jsond=jsondict)
        lines.append(lineprefix+line)
        # line = "{jsond}['response'] = True\n".format(jsond=jsondict)
        # lines.append(lineprefix+line)
        if do_oracle:
            line = "{ws_lock}.acquire()\n".format(ws_lock=self.threading_loc_name)
            lines.append(lineprefix + line)
            line = "{ws}.send(json.dumps({data_dname}))\n".format(ws=self.websocket_name, data_dname=jsondict)
            lines.append(lineprefix + line)
            line = "{msg}={ws}.recv()\n".format(msg=oracle_response_varname, ws=self.websocket_name)
            lines.append(lineprefix + line)
            line = "{ws_lock}.release()\n".format(ws_lock=self.threading_loc_name)
            lines.append(lineprefix + line)
        else:
            line = "{data_dname}['verdict']='currently_true'\n".format(data_dname=jsondict)
            lines.append(lineprefix + line)
            line = "{msg}=json.dumps({data_dname})\n".format(msg=oracle_response_varname, data_dname=jsondict)
            lines.append(lineprefix + line)
        line = "return {msg_fname}({msg_vname})\n".format(msg_fname=self.message_received_fname_service_response, msg_vname=oracle_response_varname)
        lines.append(lineprefix + line)
            
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        line = "else:\n" #ci troviamo nel caso filter con proprietà violata
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "service = {jsond}['service'] = {jsond}['service'].replace('_mon', '')\n".format(jsond=jsondict) #eliminiamo il suffisso _mon
        lines.append(lineprefix + line)
        line = "if service.endswith('/_action/send_goal'):\n" # fase send goal, impedire inizio action prima che il mess arrivi al server
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "# Block start_action before it reaches the action server.\n"
        lines.append(lineprefix + line)
        line = "response_cls = eval({srv_info}[service]['type'] + '.Response')\n".format(srv_info=self.services_info) # prepariamo risposta da inviare al client senza necessità di passare dal server
        lines.append(lineprefix + line)
        line = "filtered_response = response_cls()\n"
        lines.append(lineprefix + line)
        line = "if hasattr(filtered_response, 'accepted'): filtered_response.accepted = False\n" # modifica stato accepted in false
        lines.append(lineprefix + line) # per le action necessitiamo del campo accetped t/f
        line = "if hasattr(filtered_response, 'stamp'): filtered_response.stamp = self.get_clock().now().to_msg()\n"
        lines.append(lineprefix + line)
        line = "return filtered_response\n" #return se topic .../_action/send_goal e proprietà violata.
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        manylines = self.create_filter_cancel_goal_lines(jsondict, "service") #aggiunge logica risposta ROS2 a cancel
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        line = "raise Exception('The request violates the monitor specification, so it has been filtered out.')\n\n"
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        
        self.codegenutils.check_indent("on message func done")    
        return lines
    
    def create_on_message_service_response(self,silent,oracle_action,srv_lists,do_oracle):
        self.codegenutils.reset_indent("on message func start")
        lineprefix =''
        msg_input_var = 'message'
        header ="def {onmsgfunc}(self,{msg_input}):\n".format(onmsgfunc = self.message_received_fname_service_response.replace("self.",""),msg_input = msg_input_var)
        lines=[header]
        
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        jsondict = 'json_dict'
        line = "{jd} = json.loads({invar})\n".format(jd=jsondict,invar=msg_input_var)
        lines.append(lineprefix+line)
        line = "verdict = str({jd}['verdict'])\n".format(jd=jsondict)
        lines.append(lineprefix+line)
        line = "service = {jsondict}['service'] = {jsondict}['service'].replace('_mon', '')\n".format(jsondict=jsondict)
        lines.append(lineprefix+line)
        line = "verdict_msg = String()\n"
        lines.append(lineprefix+line)
        line = "verdict_msg.data = verdict\n"
        lines.append(lineprefix+line)
        line = "{monpubs}['verdict'].publish(verdict_msg)\n".format(monpubs=self.mon_pubs_dict_name)
        lines.append(lineprefix+line)
        line = "if verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':\n"
        lines.append(lineprefix+line)
        
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        if not do_oracle:
            line = "del {data_dname}['verdict']\n".format(data_dname=jsondict)
            lines.append(lineprefix + line)
        manylines = self.create_store_action_goal_lines(jsondict, "service")
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        manylines = self.create_finalize_action_goal_lines(jsondict, "service")
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        line = "{logging_fname}({data_dname})\n".format(logging_fname=self.logging_fname, data_dname=jsondict)
        lines.append(lineprefix+line)
        
        if not silent:
            msg = "'The response '+{data}+' is consistent, the result is returned'".format(data = msg_input_var)
            line = self.codegenutils.get_ros_info_logging_line(msg)
            lines.append(lineprefix+line)
        
        line = "return {msgdict}[{jsond}['time']]\n".format(msgdict=self.messages_dict_name, jsond=jsondict)
        lines.append(lineprefix + line)
        
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        
        # in the else for the first if 
        line = "else:\n"
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        
        # if the verdict is not true or unknown 
        line = "{logfunc}({jsond})\n".format(logfunc=self.logging_fname,jsond=jsondict)
        lines.append(lineprefix+line)
        
        if not silent:
            msg = "'The event response' + {msg} + ' is inconsistent' ".format(msg=msg_input_var)
            line = self.codegenutils.get_ros_info_logging_line(msg)
            lines.append(lineprefix+line)
        
        manylines = ["error = MonitorError()\n",
                     "error.m_service = {0}['service'].replace('_mon', '')\n".format(jsondict),
                     "error.m_time = {0}['time']\n".format(jsondict),
                     "error.m_property = {0}.get('spec', '')\n".format(jsondict),
                     ]
        lines=self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        
        if oracle_action == 'nothing':
            line = "error.m_content = str({dmsgs}[{jsond}['time']])\n".format(dmsgs=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
        else:
            manylines = ["{jsond}_copy = {jsond}.copy()\n".format(jsond=jsondict),
                         "del {jsond}_copy['service']\n".format(jsond=jsondict),
                         "del {jsond}_copy['time']\n".format(jsond=jsondict),
                         "if 'spec' in {jsond}_copy: del {jsond}_copy['spec']\n".format(jsond=jsondict),
                         
                         # "del {jsond}_copy['error']\n".format(jsond=jsondict),
                         "error.m_content = json.dumps({jsond}_copy)\n".format(jsond=jsondict)
                         ]
            lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        line = "{monpubs}['error'].publish(error)\n".format(monpubs=self.mon_pubs_dict_name)
        lines.append(lineprefix+line)

        line="error=True\n"
        lines.append(lineprefix+line)   
        lineprefix = self.codegenutils.dec_indent(lineprefix)

        line = "if {actions}[{jsond}['service']][0] != 'filter':\n".format(actions=self.actions_vname,jsond=jsondict)
        lines.append(lineprefix+line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "service = {jsond}['service'] = {jsond}['service'].replace('_mon', '')\n".format(jsond=jsondict)
        lines.append(lineprefix+line)
        
        if oracle_action == 'nothing':
            line = "return {msgdict}[{jsond}['time']]\n".format(msgdict=self.messages_dict_name, jsond=jsondict)
            lines.append(lineprefix + line)
            
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        line = "else:\n"
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        manylines = self.create_cancel_action_goal_lines(jsondict, "service")
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        line = "if service.endswith('/_action/send_goal'):\n"
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "# Revoke start_action after acceptance when the oracle rejects the response.\n"
        lines.append(lineprefix + line)
        line = "response_cls = eval({srv_info}[service]['type'] + '.Response')\n".format(srv_info=self.services_info)
        lines.append(lineprefix + line)
        line = "filtered_response = response_cls()\n"
        lines.append(lineprefix + line)
        line = "if hasattr(filtered_response, 'accepted'): filtered_response.accepted = False\n"
        lines.append(lineprefix + line)
        line = "if hasattr(filtered_response, 'stamp'): filtered_response.stamp = self.get_clock().now().to_msg()\n"
        lines.append(lineprefix + line)
        line = "return filtered_response\n"
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        manylines = self.create_filter_get_result_lines(jsondict, "service")
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        manylines = self.create_retry_cancel_goal_lines(jsondict, "service")
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        manylines = self.create_filter_cancel_goal_lines(jsondict, "service")
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        line = "raise Exception('The request violates the monitor specification, so it has been filtered out.')\n\n"
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        
        self.codegenutils.check_indent("on message func done")    
        return lines

    #aggiunge dati in services che fanno parte di actions. 
    #questi variano in base al tipo di richiesta/risposta (https://design.ros2.org/articles/actions.html#:~:text=Direction,message)
    def create_action_metadata_lines(self, data_dict_name, service_expr, payload_key): # param 1. dizionario con dati evento (req/reply) 2. nome service -> /_action/cancel_goal 3. cosa stiamo analizzando (request | response)
        lines = [
            "if {service}.endswith('/_action/send_goal'):\n".format(service=service_expr), #se evento = send goal. il pacchetto contiene goal e goal_id (guarda link sopra)
            "    {data}['event_kind'] = 'start_action_{payload}'\n".format(data=data_dict_name, payload=payload_key), #costruiamo stringa con start_action_(request|response) che permette all'oracolo di capire in che fase ci troviamo
            "    {data}['action_name'] = {service}.replace('/_action/send_goal', '')\n".format(data=data_dict_name, service=service_expr), # estrapoliamo nome acion name dal service NomeAction/_action/send_goal -> NomeAction
            "    if '{payload}' in {data} and isinstance({data}['{payload}'], dict) and 'goal_id' in {data}['{payload}']:\n".format(data=data_dict_name, payload=payload_key), # controllo che esista il payload, sia un dict, contenga goal id. l'if serve in caso di response in cui non abbiamo stessi dati
            "        {data}['goal_id'] = {data}['{payload}']['goal_id']\n".format(data=data_dict_name, payload=payload_key), #aggiungo goal_id presente nella request
            "        {data}['goal_id_key'] = json.dumps({data}['goal_id'], sort_keys=True)\n".format(data=data_dict_name), # creo chiave in formato JSON contenente il diz uuid che identifica action (necessitato dal monitor perchè non possiamo usare un diz come chiave di altro diz)
            "elif {service}.endswith('/_action/get_result'):\n".format(service=service_expr),# se evento = get_result. pacchetto contiene goal_id
            "    {data}['event_kind'] = 'get_result_{payload}'\n".format(data=data_dict_name, payload=payload_key), # necesitata da oracolo per capire fase action 
            "    {data}['action_name'] = {service}.replace('/_action/get_result', '')\n".format(data=data_dict_name, service=service_expr),
            "    if '{payload}' in {data} and isinstance({data}['{payload}'], dict) and 'goal_id' in {data}['{payload}']:\n".format(data=data_dict_name, payload=payload_key), #request e response hanno dati diversi
            "        {data}['goal_id'] = {data}['{payload}']['goal_id']\n".format(data=data_dict_name, payload=payload_key),
            "        {data}['goal_id_key'] = json.dumps({data}['goal_id'], sort_keys=True)\n".format(data=data_dict_name),
            "elif {service}.endswith('/_action/cancel_goal'):\n".format(service=service_expr),
            "    {data}['event_kind'] = 'cancel_action_{payload}'\n".format(data=data_dict_name, payload=payload_key),
            "    {data}['action_name'] = {service}.replace('/_action/cancel_goal', '')\n".format(data=data_dict_name, service=service_expr),
            "    if '{payload}' in {data} and isinstance({data}['{payload}'], dict) and 'goal_info' in {data}['{payload}'] and 'goal_id' in {data}['{payload}']['goal_info']:\n".format(data=data_dict_name, payload=payload_key),
            "        {data}['goal_id'] = {data}['{payload}']['goal_info']['goal_id']\n".format(data=data_dict_name, payload=payload_key),
            "        {data}['goal_id_key'] = json.dumps({data}['goal_id'], sort_keys=True)\n".format(data=data_dict_name),
        ]
        return lines
    #aggiunge dati in topics che fanno parte di actions.
    def create_action_topic_metadata_lines(self, data_dict_name, topic_expr):
        lines = [
            "if {topic}.endswith('/_action/status'):\n".format(topic=topic_expr), #caso status
            "    {data}['event_kind'] = 'status'\n".format(data=data_dict_name), # per oracolo per capire fase action
            "    {data}['action_name'] = {topic}.replace('/_action/status', '')\n".format(data=data_dict_name, topic=topic_expr), #estrae nome action
            "    {data}['goal_ids'] = [status['goal_info']['goal_id'] for status in {data}['status_list'] if isinstance(status, dict) and 'goal_info' in status and isinstance(status['goal_info'], dict) and 'goal_id' in status['goal_info']]\n".format(data=data_dict_name), # list of in-progress goals with goal ID, time accepted, and an enum indicating the status. più goal, più id
            "    {data}['goal_id_keys'] = [json.dumps(goal_id, sort_keys=True) for goal_id in {data}['goal_ids']]\n".format(data=data_dict_name), #trasforma in JSON elementi del diz sopra
            "elif {topic}.endswith('/_action/feedback'):\n".format(topic=topic_expr),
            "    {data}['event_kind'] = 'feedback'\n".format(data=data_dict_name),
            "    {data}['action_name'] = {topic}.replace('/_action/feedback', '')\n".format(data=data_dict_name, topic=topic_expr),
            "    if 'goal_id' in {data}:\n".format(data=data_dict_name),
            "        {data}['goal_id'] = {data}['goal_id']\n".format(data=data_dict_name),
            "        {data}['goal_id_key'] = json.dumps({data}['goal_id'], sort_keys=True)\n".format(data=data_dict_name),
        ]
        return lines

    #registra un nuovo goal nella memoria del monitor
    def create_store_action_goal_lines(self, data_dict_name, service_expr):
        lines = [
            "if {service}.endswith('/_action/send_goal') and 'response' in {data} and isinstance({data}['response'], dict):\n".format(service=service_expr, data=data_dict_name), # se messaggio intercettato = send goal. se si ha già ricevuto risposta da server.
            "    if {data}['response'].get('accepted') and 'goal_id_key' in {data}:\n".format(data=data_dict_name), # se response del server = accepted ed abbiamo un goal_id_key
            "        {goals}[{data}['goal_id_key']] = {{'action_name': {data}.get('action_name'), 'time': {data}['time'], 'cancelled': False, 'cancel_requested_by_monitor': False, 'cancel_requested_by_client': False, 'done': False}}\n".format(goals=self.action_goals_info, data=data_dict_name), # goal accettato, salviamo un dizionario con informazioni riguardanti stato e key goal_id
        ]
        return lines
    #cancella un goal già accettato dal server, quando il monitor decide che la response send_goal non è valida.
    def create_cancel_action_goal_lines(self, data_dict_name, service_expr):
        lines = [
            "if {service}.endswith('/_action/send_goal') and 'response' in {data} and isinstance({data}['response'], dict):\n".format(service=service_expr, data=data_dict_name), #service = send_goal
            "    if {data}['response'].get('accepted') and 'goal_id_key' in {data}:\n".format(data=data_dict_name), # se risposta accettata da server
            "        cancel_service = {data}.get('action_name', {service}.replace('/_action/send_goal', '')) + '/_action/cancel_goal'\n".format(data=data_dict_name, service=service_expr), #costruisce il cancel goal da inviare al server
            "        if cancel_service in {srvdict}:\n".format(srvdict=self.config_client_srvs_dict_name), # controlla se il monitor ha un client che può inviare messaggio di cancellazione
            "            cancel_request = CancelGoal.Request()\n", #crea req cancel
            "            rosidl_runtime_py.set_message_fields(cancel_request, {{'goal_info': {{'goal_id': {data}['goal_id']}}}})\n".format(data=data_dict_name), #inserisce id nella request
            "            {srvdict}[cancel_service].call_service(cancel_request)\n".format(srvdict=self.config_client_srvs_dict_name), #invia il messaggio di cancellazione
            "        if {data}['goal_id_key'] in {goals}:\n".format(data=data_dict_name, goals=self.action_goals_info), # controllo di rinforzo ma eliminabile per verificare che il goal sia in memoria
            "            {goals}[{data}['goal_id_key']]['cancel_requested_by_monitor'] = True\n".format(data=data_dict_name, goals=self.action_goals_info), #modifica stato in memoria monitor del goal
            "            {goals}[{data}['goal_id_key']]['cancelled'] = True\n".format(data=data_dict_name, goals=self.action_goals_info), #modifica stato in memoria monitor del goal
        ]
        return lines

    # quando risposta get_result viola proprietà monitor esso restituisce al client una response con stato ABORTED.    
    def create_filter_get_result_lines(self, data_dict_name, service_expr):
        lines = [
            "if {service}.endswith('/_action/get_result'):\n".format(service=service_expr), # controlla che sia una get_result
            "    response_cls = eval({srv_info}[service]['type'] + '.Response')\n".format(srv_info=self.services_info), #cpstruisce una nuova risposta filtrata da inviare
            "    filtered_response = response_cls()\n",
            "    filtered_response.status = GoalStatus.STATUS_ABORTED\n", #imposta stato in aborted
            "    return filtered_response\n", 
        ]
        return lines

    # quando risposta get_result viola proprietà monitor esso restituisce al client una response con stato ERROR_REJECTED.
    def create_filter_cancel_goal_lines(self, data_dict_name, service_expr):
        lines = [
            "if {service}.endswith('/_action/cancel_goal'):\n".format(service=service_expr),
            "    response_cls = eval({srv_info}[service]['type'] + '.Response')\n".format(srv_info=self.services_info),
            "    filtered_response = response_cls()\n",
            "    filtered_response.return_code = CancelGoal.Response.ERROR_REJECTED\n",
            "    return filtered_response\n",
        ]
        return lines
    #rimuove dalla memoria del monitor goal non più da monitorare
    #quando arriva un get_result il valore 
    def create_finalize_action_goal_lines(self, data_dict_name, service_expr):
        lines = [
            "if {service}.endswith('/_action/get_result') and 'goal_id_key' in {data} and {data}['goal_id_key'] in {goals}:\n".format(service=service_expr, data=data_dict_name, goals=self.action_goals_info), #se il service termina con get_result, nel diz è preente goal_id_key e il goal è effettivamente salvato nella lista del monitor
            "    {goals}[{data}['goal_id_key']]['done'] = True\n".format(data=data_dict_name, goals=self.action_goals_info), # lo segna come terminato (ridondate, aggiunto per chiarezza)
            "    del {goals}[{data}['goal_id_key']]\n".format(data=data_dict_name, goals=self.action_goals_info), #lo elimina
            "if {service}.endswith('/_action/cancel_goal') and 'goal_id_key' in {data} and {data}['goal_id_key'] in {goals}:\n".format(service=service_expr, data=data_dict_name, goals=self.action_goals_info), #nel caso di cancel uguale
            "    {goals}[{data}['goal_id_key']]['cancelled'] = True\n".format(data=data_dict_name, goals=self.action_goals_info),
            "    {goals}[{data}['goal_id_key']]['done'] = True\n".format(data=data_dict_name, goals=self.action_goals_info),
            "    del {goals}[{data}['goal_id_key']]\n".format(data=data_dict_name, goals=self.action_goals_info),
        ]
        return lines

    #registrare che la richiesta di cancellazione è partita dal client
    def create_track_cancel_request_lines(self, data_dict_name, service_expr):
        lines = [
            "if {service}.endswith('/_action/cancel_goal') and 'goal_id_key' in {data} and {data}['goal_id_key'] in {goals}:\n".format(service=service_expr, data=data_dict_name, goals=self.action_goals_info), #solo il client invia la cancel su quel canale. quindi sappiamo che arriva da lui senza ulteriori azioni
            "    {goals}[{data}['goal_id_key']]['cancel_requested_by_client'] = True\n".format(data=data_dict_name, goals=self.action_goals_info), #modifica stato
        ]
        return lines

    #prova l'inivio di una cancel al server
    def create_retry_cancel_goal_lines(self, data_dict_name, service_expr):
        lines = [
            "if {service}.endswith('/_action/cancel_goal') and 'goal_id' in {data}:\n".format(service=service_expr, data=data_dict_name), #controllo che finisca in canel goal la richiesta e il goal_id sia presente in memoria
            "    cancel_service = {data}.get('action_name', {service}.replace('/_action/cancel_goal', '')) + '/_action/cancel_goal'\n".format(data=data_dict_name, service=service_expr), #costruisce la cancel
            "    if cancel_service in {srvdict}:\n".format(srvdict=self.config_client_srvs_dict_name), # verifica che il monitor abbia un client per inviare il messaggio
            "        cancel_request = CancelGoal.Request()\n", #crea richiesta
            "        rosidl_runtime_py.set_message_fields(cancel_request, {{'goal_info': {{'goal_id': {data}['goal_id']}}}})\n".format(data=data_dict_name), #inserisce goal_id
            "        {srvdict}[cancel_service].call_service(cancel_request)\n".format(srvdict=self.config_client_srvs_dict_name), #la invia
        ]
        return lines

    # caso violazione topic ptova eliminazione goal coinvolti
    def create_cancel_action_topic_goal_lines(self, data_dict_name):
        lines = [
            "goal_ids = []\n", #lista vuota
            "if 'goal_id' in {data}:\n".format(data=data_dict_name), #caso feedback (1 id)
            "    goal_ids.append({data}['goal_id'])\n".format(data=data_dict_name),
            "if 'goal_ids' in {data} and isinstance({data}['goal_ids'], list):\n".format(data=data_dict_name), #caso status (+ id)
            "    goal_ids.extend([goal_id for goal_id in {data}['goal_ids'] if goal_id not in goal_ids])\n".format(data=data_dict_name), #evita duplicati nella lista
            "for goal_id in goal_ids:\n", #per ogni goal
            "    goal_id_key = json.dumps(goal_id, sort_keys=True)\n", #crea key per cercare nella lista 
            "    goal_state = {goals}.get(goal_id_key, {{'action_name': {data}.get('action_name'), 'time': {data}.get('time'), 'cancelled': False, 'cancel_requested_by_monitor': False, 'cancel_requested_by_client': False, 'done': False}})\n".format(goals=self.action_goals_info, data=data_dict_name), #recupera goal status, se presente oppure crea stato predefinito
            "    if not goal_state.get('cancelled', False):\n", #se il goal non risulta già cancellato procede
            "        cancel_service = goal_state.get('action_name', {data}.get('action_name')) + '/_action/cancel_goal'\n".format(data=data_dict_name), #crea service
            "        self.get_logger().info('Attempting topic-side cancel via ' + str(cancel_service) + ' for goal ' + str(goal_id_key))\n", #logging
            "        if cancel_service in {srvdict}:\n".format(srvdict=self.config_client_srvs_dict_name), # se client per invio esiste:
            "            cancel_request = CancelGoal.Request()\n", 
            "            rosidl_runtime_py.set_message_fields(cancel_request, {'goal_info': {'goal_id': goal_id}})\n",#inserisce goal da cancellare
            "            {srvdict}[cancel_service].call_service(cancel_request)\n".format(srvdict=self.config_client_srvs_dict_name), #invia richiesta
            "            goal_state['cancelled'] = True\n",
            "            goal_state['cancel_requested_by_monitor'] = True\n",
            "            goal_state['action_name'] = goal_state.get('action_name', {data}.get('action_name'))\n".format(data=data_dict_name),
            "            {goals}[goal_id_key] = goal_state\n".format(goals=self.action_goals_info), #salva lo stato
            "        else:\n", #non esiste il client






            
            "            self.get_logger().info('Unable to resolve cancel service ' + str(cancel_service) + ' from monitor configuration')\n",
        ]
        return lines
            
    # class init function 
    def get_variable_init_lines(self):
        lines = [
            "{dname}={{}}\n".format(dname=self.mon_pubs_dict_name),
            "{dname}={{}}\n".format(dname=self.config_pubs_dict_name),
            "{dname}={{}}\n".format(dname=self.config_subs_dict_name),
            "{dname}={{}}\n".format(dname=self.config_client_srvs_dict_name),
            "{dname}={{}}\n".format(dname=self.config_server_srvs_dict_name),
            "{dname}={{}}\n".format(dname=self.services_info),
            "{dname}={{}}\n".format(dname=self.messages_dict_name),
            "{dname}={{}}\n".format(dname=self.action_goals_info), #lista goal monitorati
            "{varname}=Lock()\n".format(varname=self.threading_loc_name),
            "{0}={1}\n".format(self.monitor_id_vname,self.mon_name_input),
            "{0}={1}\n".format(self.actions_vname,self.actions_name_input),
            "{0}={1}\n".format(self.log_name,self.log_name_input),
            "{tpinfo}={{}}\n".format(tpinfo=self.topics_info)
            ]
        
        return lines
    
    def create_init_func(self,monitor_id,subscribers,tp_lists,services,srv_lists,config_callbacks_topic,config_callbacks_service,oracle_url,oracle_port):
        self.codegenutils.reset_indent("init func start")
        lineprefix = ''
        
        header = "def __init__(self,{0},{1},{2}):\n".format(self.mon_name_input,self.log_name_input,self.actions_name_input)
        lines=[header]
        
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        
        v_init_lines = self.get_variable_init_lines()
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, v_init_lines, lineprefix)
        
        # ros init line
        rosline = "super().__init__({monname})\n".format(monname=self.monitor_id_vname.replace('/','_'))
        lines.append(lineprefix+rosline)
        
        mon_publishers = self.create_inherent_monitor_publisher_lines()
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, mon_publishers, lineprefix)

        publines = self.create_config_publishers_lines(subscribers,tp_lists)
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, publines, lineprefix)

        if self.publish_topics is not None:
            line = "{pb}={val}\n".format(pb=self.pub_topics_name,val=self.publish_topics)
            lines.append(lineprefix+line)

        srvlines = self.create_config_client_services_lines(services,srv_lists)
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, srvlines, lineprefix)
        
        tlines = self.create_topics_info_dict(tp_lists)
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, tlines, lineprefix)

        slines = self.create_services_info_dict(srv_lists)
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, slines, lineprefix)
        
        # now we also need to create the subscribers 
        # so if we have remapped a topic we need to subscribe to its remapped name 
        # if not we just subscribe to the normal topic
        slines = self.create_config_subscriber_lines(subscribers, tp_lists, config_callbacks_topic)
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, slines, lineprefix)
        slines = self.create_config_server_service_lines(services, srv_lists, config_callbacks_service)
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, slines, lineprefix)
        
        msg = "'Monitor' + {mname} + ' started and ready' ".format(mname=self.monitor_id_vname)
        line = self.codegenutils.get_ros_info_logging_line(msg)
        lines.append(lineprefix+line)
        
        msg = "'Logging at' + {logname} ".format(logname=self.log_name)
        line = self.codegenutils.get_ros_info_logging_line(msg)
        lines.append(lineprefix+line)
        
        if oracle_url != None and oracle_port != None:
            wslines = [
                "websocket.enableTrace(False)\n",
                "{ws} = websocket.WebSocket()\n".format(ws=self.websocket_name),
                "{ws}.connect('ws://{u}:{p}')\n".format(ws=self.websocket_name,u=oracle_url,p=oracle_port)
                ]
            lines = self.codegenutils.append_lines_to_list_with_prefix(lines,wslines,lineprefix)
    
            msg = "'Websocket is open'"
            line = self.codegenutils.get_ros_info_logging_line(msg)
            lines.append(lineprefix+line)
        
        self.codegenutils.check_indent("init funct end")
        return lines
    
    def create_topics_info_dict(self,tp_lists):
        lines=[]
        for t in tp_lists:
            tdict = tp_lists[t].copy()
            tdict['type'] = self.get_runtime_type_expr(tp_lists[t]) # type non fa più riferimento a una stringa, ma alla classe del messaggio. con la stringa non possiamo passarlo a ROS2 e usare le funzioni
            line = "{t_info_var}['{tname}']={tdict}\n".format(t_info_var=self.topics_info,tname=t,tdict=tdict)
            lines.append(line)
            
        return lines
    
    def create_services_info_dict(self,srv_lists):
        lines=[]
        for s in srv_lists:
            sdict = srv_lists[s].copy()
            sdict['type'] = self.get_runtime_type_expr(srv_lists[s]) # uguale ma per services
            line = "{s_info_var}['{sname}']={sdict}\n".format(s_info_var=self.services_info,sname=s,sdict=sdict)
            lines.append(line)
            
        return lines
    
    

    
    
    def create_mon_class_lines(self,topics_with_types_and_action,services_with_types_and_action,monitor_id,silent,oracle_action,oracle_url,oracle_port):
        self.codegenutils.reset_indent("mon class creation start")
        lineprefix = ''
        lines = []
        new_line = self.codegenutils.new_line
        # create the python header 
        h_line = self.codegenutils.create_python_header()
        lines.append(lineprefix+h_line)
        lines.append(new_line)
        # add the import lines  
        tp_lists= self.get_topic_and_service_msg_types(topics_with_types_and_action)
        srv_lists= self.get_topic_and_service_msg_types(services_with_types_and_action)
        subscribers = self.get_subscribers(topics_with_types_and_action)
        services = self.get_services(services_with_types_and_action)
        i_lines = self.create_import_lines(tp_lists,srv_lists)
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, i_lines, lineprefix)
        lines.append(new_line)
        # create the class header
        c_line = self.create_class_header(monitor_id)
        lines.append(lineprefix+c_line)
        lines.append(new_line)
        # increment the line prefix 
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        
        
        # do the callbacks 
        config_callbacks_topic = self.create_config_callbacks_topic(subscribers, tp_lists, silent, oracle_action, oracle_url, oracle_port)
        # for now just print the call back functions 
        for t in config_callbacks_topic:
            cblines = config_callbacks_topic[t]['lines']
            lines.append(new_line)
            lines=self.codegenutils.append_lines_to_list_with_prefix(lines, cblines, lineprefix)
        config_callbacks_service = self.create_config_callbacks_service(services, srv_lists, silent, oracle_action, oracle_url, oracle_port)
        # for now just print the call back functions 
        for s in config_callbacks_service:
            cblines = config_callbacks_service[s]['lines']
            lines.append(new_line)
            lines=self.codegenutils.append_lines_to_list_with_prefix(lines, cblines, lineprefix)
            
            
        lines.append(new_line)
        # do the init funciton 
        init_lines = self.create_init_func(monitor_id,subscribers,tp_lists,services,srv_lists,config_callbacks_topic,config_callbacks_service,oracle_url,oracle_port)
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, init_lines, lineprefix)
        lines.append(new_line)
        
        # do on message 
        lines.append(new_line)    
        if oracle_url !=None and oracle_port != None:
            on_message_lines = self.create_on_message_topic(silent, oracle_action, tp_lists)
            lines=self.codegenutils.append_lines_to_list_with_prefix(lines,on_message_lines,lineprefix)

        if srv_lists:
            on_message_lines = self.create_on_message_service_request(silent, oracle_action, srv_lists, oracle_url != None and oracle_port != None)
            lines=self.codegenutils.append_lines_to_list_with_prefix(lines,on_message_lines,lineprefix)
            on_message_lines = self.create_on_message_service_response(silent, oracle_action, srv_lists, oracle_url != None and oracle_port != None)
            lines=self.codegenutils.append_lines_to_list_with_prefix(lines,on_message_lines,lineprefix)
            
        lines.append(new_line)
        
        # do logging 
        logging_lines = self.create_logging_func()
        lines=self.codegenutils.append_lines_to_list_with_prefix(lines, logging_lines, lineprefix)
        lines.append(new_line)
        
        # lineprefix = self.codegenutils.dec_indent(lineprefix)
        self.codegenutils.check_indent("mon class creation func ")

        # srvlines = self.create_config_client_services_lines(services,srv_lists)
        
        # TBC
        
        return lines
               
    def create_service_node(self):
        lines = []
        lineprefix = ''
        line = "class ServiceNode(Node):\n"
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "def __init__(self, service_type, service_name):\n"
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "super().__init__('service_node_' + service_name.replace('/', '_'))\n"
        lines.append(lineprefix + line)
        line = "self.cli = self.create_client(service_type, service_name)\n"
        lines.append(lineprefix + line)
        line = "self.service_executor = SingleThreadedExecutor()\n"
        lines.append(lineprefix + line)
        line = "self.service_executor.add_node(self)\n"
        lines.append(lineprefix + line)
        line = "while not self.cli.wait_for_service(timeout_sec=1.0):\n"
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "self.get_logger().info('service not available, waiting again...')\n"
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        line = "def call_service(self, request):\n"
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "self.future = self.cli.call_async(request)\n"
        lines.append(lineprefix + line)
        line = "self.service_executor.spin_until_future_complete(self.future)\n"
        lines.append(lineprefix + line)
        line = "return self.future.result()\n"
        lines.append(lineprefix + line)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        lineprefix = self.codegenutils.dec_indent(lineprefix)
        return lines
            
        
    def create_mon_file_lines(self,topics_with_types_and_action,services_with_types_and_action,monitor_id,silent,oracle_action,oracle_url,oracle_port,log):
        self.codegenutils.reset_indent("mon file func")
        lineprefix = ''
        lines = self.create_mon_class_lines(topics_with_types_and_action,services_with_types_and_action,monitor_id,silent,oracle_action,oracle_url,oracle_port)
        
        mlines = self.create_main_func_lines(topics_with_types_and_action,services_with_types_and_action,log,monitor_id)
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, mlines, lineprefix)
        
        lines.append(self.codegenutils.new_line)
        mlines = self.create_python_main_lines()
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, mlines, lineprefix)
        
        self.codegenutils.check_indent("mon file creation func done")
        return lines
    
    
    def create_main_func_lines(self,topics_with_types_and_action,services_with_types_and_action,log,monitor_id):
        self.codegenutils.reset_indent("create main func ")
        lineprefix = ''
        header = "def main(args=None):\n"
        lines = [header]
        
        lineprefix=self.codegenutils.inc_indent(lineprefix)
        line = "rclpy.init(args=args)\n"
        lines.append(lineprefix+line)
        
        line = "log = '{l}'\n".format(l=log)
        lines.append(lineprefix+line)
        
        line = "actions = {}\n"
        lines.append(lineprefix+line)
        for tp in topics_with_types_and_action:
            warning = 0 
            if 'warning' in tp:
                warning = tp['warning']
            line = "actions['{tpn}']=('{act}',{w})\n".format(tpn=tp['name'],act=tp['action'],w=warning)
            lines.append(lineprefix+line)
        for srv in services_with_types_and_action:
            warning = 0 
            if 'warning' in srv:
                warning = srv['warning']
            line = "actions['{srvn}']=('{act}',{w})\n".format(srvn=srv['name'],act=srv['action'],w=warning)
            lines.append(lineprefix+line)
        
        line = "monitor = {mclassname}('{mid}',log,actions)\n".format(mclassname=self.get_mon_class_name(monitor_id),mid=monitor_id)
        
        lines.append(lineprefix+line)
        mlines = ["rclpy.spin(monitor)\n",
                "monitor.{wsname}.close()\n".format(wsname=self.websocket_name.replace("self.","")),
                "monitor.destroy_node()\n",
                "rclpy.shutdown()\n"
                ]
        lines=self.codegenutils.append_lines_to_list_with_prefix(lines, mlines, lineprefix)
        
        self.codegenutils.check_indent("create main func ")
        return lines
     
    
    
    def create_python_main_lines(self):
        self.codegenutils.reset_indent("create python main func ")
        lineprefix = ''
        lines = ["if __name__ == '__main__':\n"]
        lineprefix = self.codegenutils.inc_indent(lineprefix)
        line = "main()\n"
        lines.append(lineprefix+line)
        self.codegenutils.check_indent("create python main func ")
        return lines   
    


                  
    def create_import_lines(self, tp_lists, srv_lists):
        plain_import = ['json',
                        'yaml',
                        'websocket',
                        'sys',
                        'rclpy',
                        'rosidl_runtime_py']
        from_import = {'rclpy.node':'Node',
                     'threading':'*',
                     'rosmonitoring_interfaces.msg':'MonitorError',
                     'std_msgs.msg':'*',
                     'rclpy.executors':'SingleThreadedExecutor',
                     'rclpy.callback_groups':'MutuallyExclusiveCallbackGroup',
                     'rclpy.qos':'qos_profile_action_status_default'}
        
        ''' generate import lines for all the other message types '''
        for tp in tp_lists:
            package = tp_lists[tp]['package']
            # type = tp_lists[tp]['type']
            if not package in from_import:
                from_import[package] = '*'
        for srv in srv_lists:
            package = srv_lists[srv]['package']
            # type = srv_lists[srv]['type']
            if not package in from_import:
                from_import[package] = '*'
        
        ''' now lets generate the lines  '''
        import_lines = ["# begin imports\n"]
        for package in plain_import:
            line = "import {0}\n".format(package)
            import_lines.append(line)
            
        for package in from_import:
            line = "from {p} import {t}\n".format(p=package, t=from_import[package])
            import_lines.append(line)
        import_lines.append("# done import\n")
            
        return import_lines

  
        
    def create_config_callbacks_topic(self, subscribers, tp_lists, silent,oracle_action,oracle_url,oracle_port):
        callbacks = {}
        for topic in subscribers:
            callbacks[topic] = self.create_callback_func_topic(topic,subscribers[topic],tp_lists[topic],silent,oracle_action,oracle_url,oracle_port)
        return callbacks

    def create_config_callbacks_service(self, services, srv_lists, silent,oracle_action,oracle_url,oracle_port):
        callbacks = {}
        for service in services:
            callbacks[service] = self.create_callback_func_service(service,services[service],srv_lists[service],silent,oracle_action,oracle_url,oracle_port)
        return callbacks
    

    def create_callback_func_topic(self, tname, tinfo, tmsg_type, silent, oracle_action, oracle_url, oracle_port):
        self.codegenutils.reset_indent("callback func start")
        tpname = tname
        if tinfo['remapped']:
            tpname = self.get_remapped_name(tpname)
        lineprefix = self.codegenutils.inc_indent('')
        func_name = "callback{tname}".format(tname=tname).replace('/', '_')
        func_input_varname = 'data'
        header = "def {fname}(self,{f_input}):\n".format(fname=func_name, f_input=func_input_varname)
        lines = [header]
        
        data_dict_name = "event_dict"

        # log output if not silent
        if not silent:
            message = '"monitor has observed "+ str({0})'.format(func_input_varname)
            
            line = self.codegenutils.get_ros_info_logging_line(message)
            lines.append(lineprefix + line)
        # convert the data to send to the oracle or log
        line = "{0}= rosidl_runtime_py.message_to_ordereddict({1})\n".format(data_dict_name, func_input_varname)
        lines.append(lineprefix + line)
        
        line = "{data_dict_name}['topic']='{tname}'\n".format(data_dict_name=data_dict_name, tname=tname)
        lines.append(lineprefix + line)
        manylines = self.create_action_topic_metadata_lines(data_dict_name, "{0}['topic']".format(data_dict_name)) # aggiunge dati action quando arriva un evento topic
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        
        line = "{data_dict_name}['time']={ros_time}\n".format(data_dict_name=data_dict_name, ros_time=self.codegenutils.get_ros_time_line())
        lines.append(lineprefix + line)
        
        line = "{ws_lock}.acquire()\n".format(ws_lock=self.threading_loc_name)
        lines.append(lineprefix + line)
        # making sure we don't overwrite in the dictionary
        if oracle_action == 'nothing':
            line = "while {data_dname}['time'] in {msg_dname}:\n".format(data_dname=data_dict_name, msg_dname=self.messages_dict_name)
            lines.append(lineprefix + line)
            lineprefix = self.codegenutils.inc_indent(lineprefix)
            line = "{data_dname}['time']+=0.01\n".format(data_dname=data_dict_name)
            lines.append(lineprefix + line)
            lineprefix = self.codegenutils.dec_indent(lineprefix)
            
        do_oracle = oracle_url != None and oracle_port != None
        log_msg = "event "
        oracle_response_varname = "message"   
        # if online monitor then we need to send things to the oracle
        if do_oracle:
            log_msg += "propagated to oracle" 
            line = "{ws}.send(json.dumps({data_dname}))\n".format(ws=self.websocket_name, data_dname=data_dict_name)
            lines.append(lineprefix + line)
            if oracle_action == 'nothing':
                line = "{msgs_dname}[{data_dname}['time']] = {input_varname}\n".format(msgs_dname=self.messages_dict_name, data_dname=data_dict_name, input_varname=func_input_varname)
                lines.append(lineprefix + line)
            line = "{msg}={ws}.recv()\n".format(msg=oracle_response_varname, ws=self.websocket_name)
            lines.append(lineprefix + line)
        else:
            log_msg += "successfully logged"
            line = "{logging_fname}({data_dname})\n".format(logging_fname=self.logging_fname, data_dname=data_dict_name)
            lines.append(lineprefix + line)
            if tinfo['republish']:
                # if we need to republish then go ahead and do that
                # TODO: add line here
                line = ""
                
        line = "{ws_lock}.release()\n".format(ws_lock=self.threading_loc_name)
        lines.append(lineprefix + line)
        
        if not silent:
            line = self.codegenutils.get_ros_info_logging_line('"{0}"'.format(log_msg))
            lines.append(lineprefix + line)
            
        if do_oracle:
            line = "{msg_fname}({msg_vname})\n".format(msg_fname=self.message_received_fname_topic, msg_vname=oracle_response_varname)
            lines.append(lineprefix + line)
        
        self.codegenutils.check_indent("create callback func done")   
        return {'name':func_name, 'lines':lines}
    
    def create_callback_func_service(self, srvname, srvinfo, srvmsg_type, silent, oracle_action, oracle_url, oracle_port):
        self.codegenutils.reset_indent("callback func start")
        if srvinfo['remapped']:
            srvname = self.get_remapped_name(srvname)
        lineprefix = self.codegenutils.inc_indent('')
        func_name = "callback{srvname}".format(srvname=srvname).replace('/', '_')
        func_request = 'request'
        func_response = 'response'
        header = "def {fname}(self, {f_req}, {f_res}):\n".format(fname=func_name, f_req=func_request, f_res=func_response)
        lines = [header]
        
        data_dict_name = "event_dict"

        # log output if not silent
        if not silent:
            message = '"monitor has observed a service request with "+ str({0})'.format(func_request)
            
            line = self.codegenutils.get_ros_info_logging_line(message)
            lines.append(lineprefix + line)
        # convert the data to send to the oracle or log
        line = "{0} = {{}}\n".format(data_dict_name)
        lines.append(lineprefix + line)
        line = "{0}['request']= rosidl_runtime_py.message_to_ordereddict({1})\n".format(data_dict_name, func_request)
        lines.append(lineprefix + line)
        
        line = "{data_dict_name}['service']='{srvname}'\n".format(data_dict_name=data_dict_name, srvname=srvname.replace('_mon', ''))
        lines.append(lineprefix + line)
        manylines = self.create_action_metadata_lines(data_dict_name, "{0}['service']".format(data_dict_name), 'request')
        lines = self.codegenutils.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        
        line = "{data_dict_name}['time']={ros_time}\n".format(data_dict_name=data_dict_name, ros_time=self.codegenutils.get_ros_time_line())
        lines.append(lineprefix + line)
        
        line = "{ws_lock}.acquire()\n".format(ws_lock=self.threading_loc_name)
        lines.append(lineprefix + line)
        # making sure we don't overwrite in the dictionary
        if oracle_action == 'nothing':
            line = "while {data_dname}['time'] in {msg_dname}:\n".format(data_dname=data_dict_name, msg_dname=self.messages_dict_name)
            lines.append(lineprefix + line)
            lineprefix = self.codegenutils.inc_indent(lineprefix)
            line = "{data_dname}['time']+=0.01\n".format(data_dname=data_dict_name)
            lines.append(lineprefix + line)
            lineprefix = self.codegenutils.dec_indent(lineprefix)
            
        do_oracle = oracle_url != None and oracle_port != None
        log_msg = "event "
        oracle_response_varname = "message"   

        # line = "{data_dname}['request'] = {f_req}\n".format(data_dname=data_dict_name, f_req=func_request)
        # line = "{data_dname}['request'] = True\n".format(data_dname=data_dict_name)
        # lines.append(lineprefix+line)
        # line = "{data_dname}['result']  = {srvdict}[{srvname}].call({f_req})\n".format(srvname=srvname, srvdict=self.config_client_srvs_dict_name,msgdict=self.messages_dict_name,f_req=func_request, f_res=func_response)
        # lines.append(lineprefix+line)

        # if online monitor then we need to send things to the oracle
        if do_oracle:
            log_msg += "propagated to oracle" 
            line = "{ws}.send(json.dumps({data_dname}))\n".format(ws=self.websocket_name, data_dname=data_dict_name)
            lines.append(lineprefix + line)
            line = "{msgs_dname}[{data_dname}['time']] = {input_varname}\n".format(msgs_dname=self.messages_dict_name, data_dname=data_dict_name, input_varname=func_request)
            lines.append(lineprefix + line)
            line = "{msg}={ws}.recv()\n".format(msg=oracle_response_varname, ws=self.websocket_name)
            lines.append(lineprefix + line)
        else:
            # log_msg += "successfully logged"
            # line = "{logging_fname}({data_dname})\n".format(logging_fname=self.logging_fname, data_dname=data_dict_name)
            # lines.append(lineprefix + line)
            line = "{data_dname}['verdict']='currently_true'\n".format(data_dname=data_dict_name)
            lines.append(lineprefix + line)
            line = "{msg}=json.dumps({data_dname})\n".format(msg=oracle_response_varname, data_dname=data_dict_name)
            lines.append(lineprefix + line)
            line = "{msgs_dname}[{data_dname}['time']] = {input_varname}\n".format(msgs_dname=self.messages_dict_name, data_dname=data_dict_name, input_varname=func_request)
            lines.append(lineprefix + line)
                
        line = "{ws_lock}.release()\n".format(ws_lock=self.threading_loc_name)
        lines.append(lineprefix + line)
        
        if not silent:
            line = self.codegenutils.get_ros_info_logging_line('"{0}"'.format(log_msg))
            lines.append(lineprefix + line)
            
        line = "return {msg_fname}({msg_vname})\n".format(msg_fname=self.message_received_fname_service_request, msg_vname=oracle_response_varname)
        lines.append(lineprefix + line)
        
        self.codegenutils.check_indent("create callback func done")   
        return {'name':func_name, 'lines':lines}

        
    def create_inherent_monitor_publisher_lines(self):
        pub_types = {'error':'MonitorError', 'verdict':'String'}
        comments = "# creating the verdict and error publishers for the monitor\n"
        lines = [comments]

        for pt in pub_types:
            pubname =  "{monname}+'/monitor_{pt}'".format(monname=self.monitor_id_vname,pt=pt)
            # "{}" + "'"+'/monitor_' + pt + "'"
            ros_pub_creation_line = self.codegenutils.ros_publisher_creation_command(pubname, pub_types[pt], self.queue_size,False)
            line = "{dictname}['{pubtype}']={ros_pub_line}\n".format(dictname=self.mon_pubs_dict_name, pubtype=pt, ros_pub_line=ros_pub_creation_line)
            lines.append(line)
        comments = "# done creating monitor publishers\n\n"
        lines.append(comments)

        return lines
    
    
    def create_package_xml(self,tp_lists,location):
        
        child_to_insert_after = 11
        tree = ET.parse(location+".packagexml")
        root = tree.getroot()
        children = list(root)
        
            
        # so now we insert the relevant packages
        pkgs_so_far = []
        for t in tp_lists:
            new_field = ET.Element("exec_depend")
            if tp_lists[t]['package'] not in pkgs_so_far:
                package,sep,tail = tp_lists[t]['package'].partition('.')
                new_field .text=package
                pkgs_so_far.append(package)
            root.insert(child_to_insert_after,new_field)
        
        print ("Updated package.xml")    
        # ET.dump(root)
        tree.write(location+'package.xml')
        
    def generate_monitor_package(self,monitor_id, topics_with_types_and_action, services_with_types_and_action, log, url, port, oracle_action, silent, warning):
        monloc = 'code/monitor/monitor/'
        packageloc = 'code/monitor/'
        lines = self.create_mon_file_lines(topics_with_types_and_action, services_with_types_and_action, monitor_id, silent, oracle_action, url, port, log)
        if services_with_types_and_action:
            lines.extend(self.create_service_node())
        tp_lists = self.get_topic_and_service_msg_types(topics_with_types_and_action)
        tp_lists.update(self.get_topic_and_service_msg_types(services_with_types_and_action))
        self.codegenutils.write_lines(lines, monitor_id, monloc)
        self.create_package_xml(tp_lists, packageloc)
            
            
class LaunchFileGen(object):
    
    def create_node_node(self,fpkg,execfile,nodename,foutput="screen"):
        return ET.Element("node", pkg=fpkg,exec=execfile,name=nodename,output=foutput)
    
    def create_launch_node(self):
        return ET.Element("launch")
    
    def create_monitor_launch(self,monitor_ids,package_name):
        root = self.create_launch_node()
        for id in monitor_ids:
            nodename = sr.script_name(id)
            node_elem = self.create_node_node(package_name, nodename, nodename)
            root.insert(0,node_elem)
            
        return root
    
    def write_monitor_launch(self,monitor_ids,package_name,loc):
        root_elem = self.create_monitor_launch(monitor_ids, package_name)
        self.write_launch_file(root_elem, loc+'monitor.launch')
        
    def write_launch_file(self,root_elem,locfn):
        tree = ET.ElementTree(root_elem)
        print("writing to "+locfn)
        tree.write(locfn)
    
    
    def instrument_node_launch_files(self,nodes):
        launch_files={}
        if not nodes:
            return 
        for name in nodes:
            (package,path,topics) = nodes[name]
            if path not in launch_files:
                launch_files[path] = []
            launch_files[path].append((name,package,topics))
        for path in launch_files:
            file_name = path.replace('.launch', '_instrumented.launch')
            tree = ET.parse(path)
            launch = tree.getroot()
            for node in launch.findall('node'):
                for (name, package, topics) in launch_files[path]:
                    if node.get('name') == name and node.get('pkg') == package:
                        for topic in topics:
                            remap = ET.SubElement(node, 'remap')
                            remap.set('from', topic)
                            remap.set('to', topic + '_mon')
                        break
            self.write_launch_file(launch, file_name)    
            
               



            
