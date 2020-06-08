#include <iostream>

#include "PupilGazeScraper.hpp"
#define MSGPACK_USE_BOOST /* there's no other way to unpack gaze data */
#include <boost/variant.hpp>
#include <msgpack.hpp>

static float norm_pos[2] = {0,0};

PupilGazeScraper::PupilGazeScraper(zmq::socket_t *subscriber):
	subscriber(subscriber),
	active(false),
	gaze_smoother_x(20),
	gaze_smoother_y(20){}

/* stuff from https://stackoverflow.com/questions/24412133/deserializing-a-heterogeneous-map-with-messagepack-in-c
 * to convert msgpack to a variant_t
 */
typedef boost::make_recursive_variant<
  std::string,
  std::map<boost::recursive_variant_, boost::recursive_variant_>,
  std::vector<boost::recursive_variant_>,
  int,
  double
  >::type variant_t;

namespace msgpack {

	// Convert from msgpacl::object to variant_t.
	inline variant_t& operator>>(object const& o, variant_t& v) {
	    switch(o.type) {
	    case type::MAP:
	        v = std::map<variant_t, variant_t>();
	        o.convert(boost::get<std::map<variant_t, variant_t> >(&v));
	        break;
	    case type::ARRAY:
	        v = std::vector<variant_t>();
	        o.convert(boost::get<std::vector<variant_t> >(&v));
	        break;
	    case type::POSITIVE_INTEGER:
	        v = int();
	        o.convert(boost::get<int>(&v));
	        break;
	    case type::DOUBLE:
	        v = double();
	        o.convert(boost::get<double>(&v));
	        break;
	    case type::RAW:
	        v = std::string();
	        o.convert(boost::get<std::string>(&v));
	        break;
	    default:
	        break;
	    }
	    return v;
	}
} // namespace msgpack

void retrieveGaze(variant_t const& v) {
	struct retriever:boost::static_visitor<void> {
		void operator()(int value) const {
		}
		void operator()(double value) const {
		}
		void operator()(std::string const& value) const {
		}
		void operator()(std::vector<variant_t> const& value) const {
		}

		/* And this is why I will never use msgpack ever again with C++ */
		void operator()(std::map<variant_t, variant_t> const& value) const {
			for (auto const &v : value) {
				boost::apply_visitor(*this, v.first);
				/* 0 is string */
				if(v.first.which() == 0){
					std::string key = boost::get<std::string>(v.first);
					if(key == "norm_pos"){
						/* as a fact, gaze topic has the norm_pos in a vector so take it and get out */
						boost::apply_visitor(*this, v.second);
						std::vector<variant_t> gaze_v = boost::get<std::vector<variant_t>>(v.second);
						std::vector<double> gaze_d;
						for(auto const &g: gaze_v){
							gaze_d.push_back(boost::get<double>(g));
						}
						norm_pos[0] = gaze_d[0];
						norm_pos[1] = 1 - gaze_d[1];
						break;
					}
				}
			}
		}
	};
	boost::apply_visitor(retriever(), v);
}

void PupilGazeScraper::run(){
	state_mutex.lock();
	active = true;
	state_mutex.unlock();

	zmq::message_t gaze_msg;
	char *sub_buffer;
	while(active){
		subscriber->recv(&gaze_msg);

		/* see if there's more data to get, if yes then get the data */
		size_t remaining_messages = 0;
		size_t option_length = sizeof(remaining_messages);
		subscriber->getsockopt(ZMQ_RCVMORE,&remaining_messages,&option_length);
		if(remaining_messages){
			gaze_msg.rebuild();
			subscriber->recv(&gaze_msg);
			sub_buffer = (char *) malloc(sizeof(char)*gaze_msg.size());
			memcpy(sub_buffer,gaze_msg.data(),gaze_msg.size());

			/* de serialize the gaze data */
			msgpack::unpacked deserial;
			msgpack::unpack(&deserial,sub_buffer,gaze_msg.size());
			msgpack::object gaze_obj = deserial.get();

			variant_t gaze_var;
			gaze_obj.convert(&gaze_var);
			retrieveGaze(gaze_var);

			storeGaze(norm_pos[0],norm_pos[1]);
			free(sub_buffer);
		}
	}
}

void PupilGazeScraper::stop(){
	state_mutex.lock();
	active = false;
	state_mutex.unlock();
}

void PupilGazeScraper::storeGaze(float x, float y){
	state_mutex.lock();
	gaze_smoother_x.push_back(x);
	gaze_smoother_y.push_back(y);
	state_mutex.unlock();
}

GazePoint PupilGazeScraper::getGazePoint(){
	std::lock_guard<std::mutex> lock(state_mutex);
	GazePoint averagedGaze;
	averagedGaze.x = gaze_smoother_x.getAverage();
	averagedGaze.y = gaze_smoother_y.getAverage();
	return averagedGaze;
}
