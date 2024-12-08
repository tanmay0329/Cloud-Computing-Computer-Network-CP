import { useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import axios from 'axios';

const AlertDisplay = () => {
    const navigate = useNavigate();

    const [alertData, setAlertData] = useState([]);

    useEffect(() => {
        async function handleAllAlert() {
            try {
                const resp = await axios.get("https://glacial-lake-outburst-floods-system.onrender.com/api/all-alert-data");
                console.log(resp.data.alertForm);
                setAlertData(resp.data.alertForm);
            } catch (error) {
                console.log("Error occurred while getting all alert form info = ", error);
            }
        }
        handleAllAlert();
    }, []);

    return (
        <div className='bg-slate-900'>
                <button onClick={()=>navigate("/main")} className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded m-5">
                    Main Menu
                </button>
            <div className='grid grid-cols-4 bg-slate-900 text-white font-light  h-screen '>
                <div className='w-full p-4 sm:w-3/4'>
                    {alertData.map((item) => (
                        <div key={item._id} className='bg-slate-700 rounded-lg shadow-md p-4 my-4 text-white'>
                            <h3 className='text-lg font-semibold text-blue-400'>Alert from: {item.name}</h3>
                            <p className='mt-2 text-sm'>
                                <span className='font-medium'>Contact Info:</span> {item.contactInfo}
                            </p>
                            <p className='mt-2 text-sm'>
                                <span className='font-medium'>Email:</span> {item.email}
                            </p>
                            <p className='mt-2 text-sm'>
                                <span className='font-medium'>Alert Message:</span> {item.alertMsg}
                            </p>
                        </div>
                    ))}
                </div>
            </div>
        </div>
    );
};

export default AlertDisplay;
