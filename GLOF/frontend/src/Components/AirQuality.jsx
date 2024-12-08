import { useState, useEffect } from 'react';
import axios from 'axios';
import { Pie } from 'react-chartjs-2';
import Chart from 'chart.js/auto';
import Header from './Header';
import { Link } from 'react-router-dom';


const AirQuality = () => {
    const [weatherData, setWeatherData] = useState(null);
    const [error, setError] = useState(null);
    const [place, setPlace] = useState("")


    const handleWeatherData = async () => {
        try {
            console.log(place)
            const resp = await axios.get(`https://glacial-lake-outburst-floods-system.onrender.com/api/weatherData/${place}`);
            setWeatherData(resp.data);
            setError(null); // Clear any previous errors
        } catch (err) {
            console.log("Error fetching weather data: ", err);
            setError("Failed to fetch weather data. Please try again later.");
        }
    };
    const chartData = {
        labels: ['CO', 'NO2', 'O3', 'SO2', 'PM2.5', 'PM10'],
        datasets: [
            {
                label: 'Air Quality',
                data: [
                    weatherData?.current.air_quality.co || 0,
                    weatherData?.current.air_quality.no2 || 0,
                    weatherData?.current.air_quality.o3 || 0,
                    weatherData?.current.air_quality.so2 || 0,
                    weatherData?.current.air_quality.pm2_5 || 0,
                    weatherData?.current.air_quality.pm10 || 0
                ],
                backgroundColor: [
                    'rgba(255, 99, 132, 0.2)',
                    'rgba(54, 162, 235, 0.2)',
                    'rgba(255, 206, 86, 0.2)',
                    'rgba(75, 192, 192, 0.2)',
                    'rgba(153, 102, 255, 0.2)',
                    'rgba(255, 159, 64, 0.2)'
                ],
                borderColor: [
                    'rgba(255, 99, 132, 1)',
                    'rgba(54, 162, 235, 1)',
                    'rgba(255, 206, 86, 1)',
                    'rgba(75, 192, 192, 1)',
                    'rgba(153, 102, 255, 1)',
                    'rgba(255, 159, 64, 1)'
                ],
                borderWidth: 1,
            },
        ],
    };

    useEffect(() => {
        // Clean up chart instance if needed
        return () => {
            Chart.getChart("chart")?.destroy();
        };
    }, [weatherData]);

    return (
        <div>
            <Header />
            <input className='border-2 border-black py-2 px-4 m-4 bg-slate-400 rounded-md' type="text" value={place} onChange={(e)=>setPlace(e.target.value)}/>

            <button className='border py-2 px-4 border-black rounded-lg bg-slate-900 text-slate-200 mx-1' onClick={handleWeatherData}>Submit</button>
            <Link to="/weatherData"><button className='border py-2 px-4 border-black rounded-lg bg-slate-900 text-slate-200 mx-1' >Wheater</button></Link>
            <Link to="/temp-humidity"><button className='border py-2 px-4 border-black rounded-lg bg-slate-900 text-slate-200 mx-1' >Humidity And Temperature</button></Link>
            {error && <p>{error}</p>}
            {weatherData && (
                <div className='flex flex-col justify-center items-center'>
                    <h3>Weather Conditions:</h3>
                    <p>Temperature: {weatherData.current.temp_c}°C</p>
                    <p>Condition: {weatherData.current.condition.text}</p>
                    <div className='w-[300px] h-[300px]'>

                    <Pie data={chartData} />
                    </div>
                </div>
            )}
        </div>
    );
};

export default AirQuality;
