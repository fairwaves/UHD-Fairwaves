//
// Copyright 2015-2015 Fairwaves LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "umtrx_impl.hpp"
#include "umtrx_log_adapter.hpp"
#include <uhd/types/sensors.hpp>
#include <uhd/types/ranges.hpp>
#include <boost/asio.hpp>

using namespace uhd;
using namespace uhd::usrp;
namespace asio = boost::asio;

/*!
 * Querying sensors using the status monitor server.
 * Requests are encoded in JSON and end in a newline.
 * Responses are encoded in JSON and end in a newline.
 *
 * Start UmTRX driver with status_port set in args
 * ./some_application --args="status_port=12345"
 *
 * import json
 * import socket
 * s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
 * s.connect(("localhost", 12345))
 * f = s.makefile() #gives us readline()
 *
 * #list branches in the property tree
 * s.send(json.dumps(dict(action='LIST', path='/mboards/0/sensors'))+'\n')
 * print json.loads(f.readline())
 * {u'result': [u'tempA']}
 *
 * #check if the specified path exists
 * s.send(json.dumps(dict(action='HAS', path='/mboards/0/sensors/tempA'))+'\n')
 * print json.loads(f.readline())
 * {u'result': u'true'}
 *
 * #get the value of a tree entry, types can be BOOL, INT, DOUBLE, COMPLEX, SENSOR, RANGE
 * s.send(json.dumps(dict(action='GET', path='/mboards/0/sensors/tempA', type='SENSOR'))+'\n')
 * print json.loads(f.readline())
 * {u'result': {u'unit': u'C', u'name': u'TempA', u'value': u'61.625000'}}
 *
 * #set the value of a tree entry, types can be BOOL, INT, DOUBLE, COMPLEX
 * s.send(json.dumps(dict(action='SET', path='/mboards/0/dboards/A/rx_frontends/0/freq/value', type='DOUBLE', value=1e9))+'\n')
 * print json.loads(f.readline())
 * {} #empty response means no error
 *
 * #in case of COMPLEX 'value' is an array of [real, imag] values
 * s.send(json.dumps(dict(action='SET', path='/mboards/0/rx_frontends/A/dc_offset/value', type='COMPLEX', value=[0.1, 0.0]))+'\n')
 * print json.loads(f.readline())
 * {} #empty response means no error
 */

void umtrx_impl::status_monitor_start(const uhd::device_addr_t &device_addr)
{
    if (device_addr.has_key("status_port"))
    {
        UHD_MSG(status) << "Creating TCP monitor on port " << device_addr.get("status_port") << std::endl;
        _server_query_tcp_acceptor.reset(new asio::ip::tcp::acceptor(
            _server_query_io_service, asio::ip::tcp::endpoint(asio::ip::address::from_string("127.0.0.1"), device_addr.cast<int>("status_port", 0))));
        _server_query_task = task::make(boost::bind(&umtrx_impl::server_query_handler, this));
    }
    _status_monitor_task = task::make(boost::bind(&umtrx_impl::status_monitor_handler, this));
}

void umtrx_impl::status_monitor_stop(void)
{
    _status_monitor_task.reset();
    _server_query_task.reset();
}

static bool wait_read_sockfd(const int sockfd, const size_t timeoutMs)
{
    //setup timeval for timeout
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeoutMs*1000;

    //setup rset for timeout
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(sockfd, &rset);

    //call select with timeout on receive socket
    return ::select(sockfd+1, &rset, NULL, NULL, &tv) > 0;
}

void umtrx_impl::status_monitor_handler(void)
{
    //TODO read the sensors and react...
    //read_dc_v, etc...
    //UHD_MSG(status) << this->read_temp_c("A").to_pp_string() << std::endl;

    //TODO shutdown frontend when temp > thresh
    //ctrl->set_rx_enabled(false);
    //ctrl->set_tx_enabled(false);

    //this sleep defines the polling time between status checks
    //when the handler completes, it will be called again asap
    //if the task is canceled, this sleep in interrupted for exit
    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
}

void umtrx_impl::server_query_handler(void)
{
    //accept the client socket (timeout is 100 ms, task is called again)
    if (not wait_read_sockfd(_server_query_tcp_acceptor->native_handle(), 100)) return;
    boost::shared_ptr<asio::ip::tcp::socket> socket(new asio::ip::tcp::socket(_server_query_io_service));
    _server_query_tcp_acceptor->accept(*socket);

    //create a new thread to handle the client
    boost::thread handler(boost::bind(&umtrx_impl::client_query_handle, this, socket));
    handler.detach();
}

void umtrx_impl::client_query_handle(boost::shared_ptr<boost::asio::ip::tcp::socket> socket)
{
    while (not boost::this_thread::interruption_requested())
    {
        boost::property_tree::ptree request, response;

        //receive the request in JSON markup
        boost::asio::streambuf requestBuff;
        try
        {
            boost::asio::read_until(*socket, requestBuff, "\n");
            std::istream is(&requestBuff);
            boost::property_tree::read_json(is, request);
        }
        catch (const std::exception &ex)
        {
            if (requestBuff.size() == 0) return; //client ended
            response.put("error", "request parser error: " + std::string(ex.what()));
        }

        //handle the request
        try
        {
            this->client_query_handle1(request, response);
        }
        catch (const std::exception &ex)
        {
            response.put("error", "failed to handle request: " + std::string(ex.what()));
        }

        //send the response
        boost::asio::streambuf responseBuff;
        std::ostream os(&responseBuff);
        try
        {
            boost::property_tree::write_json(os, response, false/*not pretty required*/);
            boost::asio::write(*socket, responseBuff);
        }
        catch (const std::exception &ex)
        {
            UHD_MSG(error) << "client_query_handle send response failed, exit client thread: " << ex.what() << std::endl;
            return;
        }
    }
}

void umtrx_impl::client_query_handle1(const boost::property_tree::ptree &request, boost::property_tree::ptree &response)
{
    const std::string action = request.get("action", "");
    const std::string path = request.get("path", "");
    if (response.count("error") != 0)
    {
        //already in error
    }
    else if (path.empty())
    {
        response.put("error", "path field not specified");
    }
    else if (action.empty())
    {
        response.put("error", "action field not specified: GET, SET, HAS, LIST");
    }
    else if (action == "GET")
    {
        const std::string type = request.get("type", "");
        if (type.empty()) response.put("error", "type field not specified: STRING, BOOL, INT, DOUBLE, COMPLEX, SENSOR, RANGE");
        else if (type == "STRING") response.put("result", _tree->access<std::string>(path).get());
        else if (type == "BOOL") response.put("result", _tree->access<bool>(path).get());
        else if (type == "INT") response.put("result", _tree->access<int>(path).get());
        else if (type == "DOUBLE") response.put("result", _tree->access<double>(path).get());
        else if (type == "COMPLEX")
        {
            boost::property_tree::ptree result;
            boost::property_tree::ptree ptree_i, ptree_q;
            const std::complex<double> c = _tree->access<std::complex<double> >(path).get();
            ptree_i.put("", c.real());
            ptree_q.put("", c.imag());
            result.push_back(std::make_pair("", ptree_i));
            result.push_back(std::make_pair("", ptree_q));
            response.add_child("result", result);
        }
        else if (type == "SENSOR")
        {
            boost::property_tree::ptree result;
            const sensor_value_t sensor = _tree->access<sensor_value_t>(path).get();
            result.put("name", sensor.name);
            result.put("value", sensor.value);
            result.put("unit", sensor.unit);
            response.add_child("result", result);
        }
        else if (type == "RANGE")
        {
            boost::property_tree::ptree result;
            BOOST_FOREACH(const range_t &range, _tree->access<meta_range_t>(path).get())
            {
                boost::property_tree::ptree rangeData;
                rangeData.put("start", range.start());
                rangeData.put("stop", range.stop());
                rangeData.put("step", range.step());
                result.push_back(std::make_pair("", rangeData));
            }
            response.add_child("result", result);
        }
        else response.put("error", "unknown type: " + type);
    }
    else if (action == "SET")
    {
        const std::string type = request.get("type", "");
        if (type.empty()) response.put("error", "type field not specified: STRING, BOOL, INT, DOUBLE, COMPLEX");
        else if (type == "STRING") _tree->access<std::string>(path).set(request.get<std::string>("value"));
        else if (type == "BOOL") _tree->access<bool>(path).set(request.get<bool>("value"));
        else if (type == "INT") _tree->access<int>(path).set(request.get<int>("value"));
        else if (type == "DOUBLE") _tree->access<double>(path).set(request.get<double>("value"));
        else if (type == "COMPLEX")
        {
            boost::property_tree::ptree value = request.get_child("value");
            double i = value.front().second.get<double>("");
            double q = value.back().second.get<double>("");
            _tree->access<std::complex<double> >(path).set(std::complex<double>(i, q));
        }
        else response.put("error", "unknown type: " + type);
    }
    else if (action == "HAS")
    {
        response.put("result", _tree->exists(path));
    }
    else if (action == "LIST")
    {
        boost::property_tree::ptree result;
        BOOST_FOREACH(const std::string &branchName, _tree->list(path))
        {
            boost::property_tree::ptree branchData;
            branchData.put("", branchName);
            result.push_back(std::make_pair("", branchData));
        }
        response.add_child("result", result);
    }
    else
    {
        response.put("error", "unknown action: " + action);
    }
}
