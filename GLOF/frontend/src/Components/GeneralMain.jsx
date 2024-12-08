import { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import axios from 'axios';

const GeneralMain = () => {
  const [showForm, setShowForm] = useState(false);
  const [formData, setFormData] = useState({
    name: "",
    contactInfo: "",
    email: "",
    alertMsg: ""
  });

  const [notify, setNotify] = useState(false);

  const navigate = useNavigate();

  const runNotification = () =>{
    setNotify(true)
    setTimeout(() => {
      console.log("hello")
      setNotify(false)
      
    }, 4000);
  }


  const handleInputChange = (e) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value
    });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    try {
      const resp = await axios.post('https://glacial-lake-outburst-floods-system.onrender.com/api/send-alert-form', formData);
      console.log("Alert sent: ", resp);
      // Reset the form and hide it
      setFormData({ name: "", contactInfo: "", email: "", alertMsg: "" });
      setShowForm(false);
      runNotification()

    } catch (error) {
      console.log("Error sending alert", error);
    }
  };


  return (
    <div>
      <div className='bg-slate-900 text-white font-light flex flex-col justify-center items-center h-screen'>
        {notify && (
        <div className="bg-blue-100 border-t border-b border-blue-500 text-blue-700 px-4 py-3 absolute z-10 mb-96 animate-pulse" role="alert">
          <p className="font-bold">Alert Form Submitted</p>
          <p className="text-sm">Authorities Are Notfied, Take care of your loved Ones.</p>
        </div>
        )}
        <button onClick={() => navigate("/weatherData")} className='bg-slate-700 py-2 px-4 my-2 text-lg rounded-lg w-[20%]'>
          Check Real-Time Weather
        </button>
        <button onClick={() => navigate("/temp-humidity")} className='bg-slate-700 py-2 px-4 my-2 text-lg rounded-lg w-[20%]'>
          Humidity And Temperature
        </button>
        <button onClick={() => navigate("/airQuality")} className='bg-slate-700 py-2 px-4 my-2 text-lg rounded-lg w-[20%]'>
          Air Quality
        </button>
        <button onClick={() => setShowForm(true)} className='bg-slate-700 py-2 px-4 my-2 text-lg rounded-lg w-[20%]'>
          Pop Form for Alert
        </button>
      </div>

      {/* Form modal */}
      {showForm && (
        <div className='fixed inset-0 bg-black bg-opacity-50 flex justify-center items-center'>
          <div className='bg-white p-6 rounded-lg shadow-lg w-[90%] sm:w-[40%]'>
            <h2 className='text-lg font-semibold mb-4'>Send Alert</h2>
            <form onSubmit={handleSubmit}>
              <div className='mb-4'>
                <label className='block text-sm font-medium text-gray-700'>Name</label>
                <input
                  type="text"
                  name="name"
                  value={formData.name}
                  onChange={handleInputChange}
                  className='mt-1 p-2 block w-full border border-gray-300 rounded-md shadow-sm focus:ring-blue-500 focus:border-blue-500'
                  required
                />
              </div>
              <div className='mb-4'>
                <label className='block text-sm font-medium text-gray-700'>Contact Info</label>
                <input
                  type="text"
                  name="contactInfo"
                  value={formData.contactInfo}
                  onChange={handleInputChange}
                  className='mt-1 p-2 block w-full border border-gray-300 rounded-md shadow-sm focus:ring-blue-500 focus:border-blue-500'
                  required
                />
              </div>
              <div className='mb-4'>
                <label className='block text-sm font-medium text-gray-700'>Email</label>
                <input
                  type="email"
                  name="email"
                  value={formData.email}
                  onChange={handleInputChange}
                  className='mt-1 p-2 block w-full border border-gray-300 rounded-md shadow-sm focus:ring-blue-500 focus:border-blue-500'
                  required
                />
              </div>
              <div className='mb-4'>
                <label className='block text-sm font-medium text-gray-700'>Alert Message</label>
                <textarea
                  name="alertMsg"
                  value={formData.alertMsg}
                  onChange={handleInputChange}
                  className='mt-1 p-2 block w-full border border-gray-300 rounded-md shadow-sm focus:ring-blue-500 focus:border-blue-500'
                  required
                ></textarea>
              </div>
              <div className='flex justify-end'>
                <button type="button" onClick={() => setShowForm(false)} className='bg-gray-500 text-white py-2 px-4 rounded-lg mr-2'>
                  Cancel
                </button>
                <button type="submit" className='bg-blue-600 text-white py-2 px-4 rounded-lg'>
                  Send
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </div>
  );
};

export default GeneralMain;
